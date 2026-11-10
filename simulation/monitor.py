#!/usr/bin/env python3
"""Read-only ROS 2 sensor monitor and APF recommendation dashboard.

No autopilot publishers, actuation services, arming or flight-command code is instantiated.
The ROS node may expose standard rosout/parameter metadata publishers.
Run with ROS Jazzy/px4_msgs sourced and the project src on PYTHONPATH.
"""
import argparse
from collections import deque
from dataclasses import asdict, replace
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit
import uuid

from uav_nav.core import (Config,ScanSample,pose_from_px4,build_observation,
                    apf_velocity,limit_velocity,MonitorWriter)


class StatusStore:
    """Atomic immutable snapshots shared by the ROS and HTTP threads."""
    def __init__(self):
        self._lock=threading.Lock()
        self._encoded=json.dumps({'schema_version':'uav-nav.live.v1','ready':False,
                       'state':'waiting_for_first_monitor_tick','flight_command_sent':False})
        self._written_at=None

    def update(self,payload,now):
        # Prove that the actual snapshot can be encoded as standards-compliant JSON.
        encoded=json.dumps(payload,allow_nan=False)
        with self._lock:
            self._encoded=encoded
            self._written_at=now

    def snapshot(self):
        with self._lock:
            payload=json.loads(self._encoded)
            written_at=self._written_at
        payload['monitor_sample_age_s']=max(0.,time.monotonic()-written_at) if written_at is not None else None
        payload['http_served_at_utc']=datetime.now(timezone.utc).isoformat()
        return payload


def make_handler(store,page):
    """Only two read-only routes. Never expose a directory or write endpoint."""
    page=page.encode('utf-8') if isinstance(page,str) else page
    class Handler(BaseHTTPRequestHandler):
        def send_payload(self,status,data,content_type):
            self.send_response(status)
            self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma','no-cache')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path=urlsplit(self.path).path
            if path=='/':
                self.send_payload(200,page,'text/html; charset=utf-8')
            elif path=='/api/status':
                self.send_payload(200,json.dumps(store.snapshot(),allow_nan=False).encode(),
                                  'application/json; charset=utf-8')
            else:
                self.send_payload(404,b'{"error":"not_found"}','application/json')

        def do_POST(self):
            self.send_payload(405,b'{"error":"read_only_monitor"}','application/json')
        do_PUT=do_POST
        do_DELETE=do_POST
        do_PATCH=do_POST

        def log_message(self,*_):
            pass
    return Handler


def frequency(history,now,window=5.):
    while history and history[0] < now-window:
        history.popleft()
    if len(history)<2:
        return 0.
    # Include wall time since the last reception, so a paused stream decays.
    duration=now-history[0]
    return (len(history)-1)/duration if duration>0 else 0.


class SourceProgress:
    """Track progress within one producer clock without subtracting clock origins."""
    def __init__(self):
        self.stamp=None
        self.progress_received_at=None
        self.resets=0

    def received(self,stamp,now):
        if not math.isfinite(stamp):
            return
        if self.stamp is None or stamp!=self.stamp:
            if self.stamp is not None and stamp<self.stamp:
                self.resets+=1
            self.stamp=stamp
            self.progress_received_at=now

    def age(self,now):
        return now-self.progress_received_at if self.progress_received_at is not None else None


def preserve_previous_logs(directory):
    names=('monitor.csv','monitor.jsonl','monitor_summary.json','status.json','run_summary.json')
    existing=[directory/name for name in names if (directory/name).exists()]
    if not existing:
        return
    archive=directory/'previous_runs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    archive.mkdir(parents=True,exist_ok=True)
    for file in existing:
        target=archive/file.name
        if target.exists():
            target=archive/(file.stem+'-'+uuid.uuid4().hex[:8]+file.suffix)
        file.replace(target)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8086)
    parser.add_argument('--goal',type=float,nargs=2,default=(5.,0.),metavar=('EAST','NORTH'))
    parser.add_argument('--duration',type=float,default=0.,help='0: continuous; positive: stop after these seconds')
    args=parser.parse_args()
    if not 1 <= args.port <= 65535 or not all(math.isfinite(v) for v in args.goal):
        parser.error('Invalid port or finite ENU goal.')
    if not math.isfinite(args.duration) or args.duration<0:
        parser.error('--duration must be a nonnegative finite number.')
    # Imports remain lazy so HTTP and clock contracts can be tested without ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
    from sensor_msgs.msg import LaserScan
    from px4_msgs.msg import VehicleOdometry,VehicleStatus

    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    preserve_previous_logs(output)
    store=StatusStore()
    page=Path(__file__).with_name('dashboard.html').read_text()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(store,page))
    server.daemon_threads=True
    thread=threading.Thread(target=server.serve_forever,daemon=True,name='uav-readonly-http')
    thread.start()

    class PassiveMonitor(Node):
        def __init__(self):
            super().__init__('uav_passive_monitor')
            self.cfg=Config()
            self.scan=self.pose=None
            self.scan_frame_id=None
            self.pose_error=None
            self.vehicle=None
            self.vehicle_received_at=None
            self.started=time.monotonic()
            self.last_tick=self.started
            self.previous=(0.,0.)
            self.run_id=uuid.uuid4().hex
            self.log=MonitorWriter(output)
            self.history={name:deque() for name in ('scan','odometry','vehicle_status')}
            self.counts={name:0 for name in self.history}
            self.progress={name:SourceProgress() for name in ('scan','odometry')}
            self.zero_samples=0
            self.rejected_samples=0
            self.latest=None
            qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(LaserScan,'/uav/scan',self.scan_received,qos)
            self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.odometry_received,qos)
            self.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',self.status_received,qos)
            self.create_timer(.1,self.tick)
            self.get_logger().info(f'PASSIVE monitor http://127.0.0.1:{args.port}; no autopilot publishers; output={output}')

        def received(self,name,now):
            self.counts[name]+=1
            self.history[name].append(now)

        def scan_received(self,msg):
            now=time.monotonic()
            self.received('scan',now)
            source=float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
            self.progress['scan'].received(source,now)
            self.scan_frame_id=str(msg.header.frame_id)
            self.scan=ScanSample(tuple(float(x) for x in msg.ranges),float(msg.angle_min),
                                 float(msg.angle_increment),float(msg.range_min),float(msg.range_max),now,source)

        def odometry_received(self,msg):
            now=time.monotonic()
            self.received('odometry',now)
            source=float(msg.timestamp_sample if msg.timestamp_sample else msg.timestamp)*1e-6
            self.progress['odometry'].received(source,now)
            try:
                self.pose=pose_from_px4(msg.position,msg.velocity,msg.q,now,source,
                                        pose_frame=int(msg.pose_frame),velocity_frame=int(msg.velocity_frame))
                self.pose_error=None
            except (ValueError,TypeError) as error:
                self.pose=None
                self.pose_error=str(error)

        def status_received(self,msg):
            now=time.monotonic()
            self.received('vehicle_status',now)
            self.vehicle_received_at=now
            self.vehicle={'arming_state':int(msg.arming_state),'armed':int(msg.arming_state)==int(msg.ARMING_STATE_ARMED),
                          'nav_state':int(msg.nav_state),'failsafe':bool(msg.failsafe)}

        def tick(self):
            now=time.monotonic()
            dt=max(.001,min(.5,now-self.last_tick))
            self.last_tick=now
            observation=build_observation(self.scan,self.pose,tuple(args.goal),now=now,cfg=self.cfg)
            extra=[]
            for name,tracker in self.progress.items():
                age=tracker.age(now)
                limit=self.cfg.max_scan_age if name=='scan' else self.cfg.max_pose_age
                if age is not None and age>limit:
                    extra.append(name+'_source_timestamp_not_advancing')
            if extra:
                observation=replace(observation,ready=False,reasons=tuple(dict.fromkeys((*observation.reasons,*extra))))
            candidate=apf_velocity(observation,self.cfg)
            action=limit_velocity(candidate,self.previous,dt,cfg=self.cfg,
                                  ready=observation.ready,min_range_m=observation.min_range_m)
            self.previous=action.velocity_enu
            self.zero_samples+=int(action.velocity_enu==(0.,0.))
            self.rejected_samples+=int(not observation.ready)
            self.log.write(now,self.pose,observation,action)
            rates={name:frequency(history,now) for name,history in self.history.items()}
            payload={
                'schema_version':'uav-nav.live.v1','run_id':self.run_id,
                'generated_at_utc':datetime.now(timezone.utc).isoformat(),
                'elapsed_s':now-self.started,'sample_count':self.log.count,
                'ready_samples':self.log.ready_count,'rejected_samples':self.rejected_samples,
                'zero_recommendation_samples':self.zero_samples,
                'ready':observation.ready,
                'mode':'PASSIVE_MONITOR_NO_AUTOPILOT_PUBLICATIONS','flight_command_sent':False,
                'goal_enu_m':list(args.goal),'rates_hz':rates,'rate_window_s':5.,'message_counts':dict(self.counts),
                'pose':asdict(self.pose) if self.pose else None,'pose_error':self.pose_error,
                'scan':{'ray_count':len(self.scan.ranges),'frame_id':self.scan_frame_id,'range_min_m':self.scan.range_min,
                        'range_max_m':self.scan.range_max,'source_timestamp_s':self.scan.source_timestamp}
                        if self.scan else None,
                'observation':asdict(observation),
                'recommendation':{'requested_enu_m_s':list(candidate),**asdict(action)},
                'vehicle':({**self.vehicle,'received_age_s':now-self.vehicle_received_at}
                           if self.vehicle is not None else None),
                'source_progress_age_s':{name:tracker.age(now) for name,tracker in self.progress.items()},
                'source_clock_reset_counts':{name:tracker.resets for name,tracker in self.progress.items()},
                'topics':{'scan':'/uav/scan','odometry':'/fmu/out/vehicle_odometry',
                          'vehicle_status':'/fmu/out/vehicle_status_v1'},
                'config':asdict(self.cfg),
                'scope':'PX4/Gazebo sensor monitoring with dry-run APF recommendations; no offboard flight loop',
            }
            store.update(payload,now)
            temporary=output/'status.json.tmp'
            temporary.write_text(json.dumps(payload,indent=2,allow_nan=False)+'\n')
            os.replace(temporary,output/'status.json')
            self.latest=payload
            if args.duration and now-self.started>=args.duration:
                rclpy.shutdown()

        def close(self):
            summary=self.log.close()
            summary.update({'run_id':self.run_id,'message_counts':self.counts,
                            'elapsed_s':time.monotonic()-self.started,
                            'zero_recommendation_samples':self.zero_samples,
                            'rejected_samples':self.rejected_samples,
                            'flight_command_sent':False,'explicit_application_publishers_created':0,
                            'standard_ros_metadata_publishers_may_exist':True,
                            'source_clock_reset_counts':{name:t.resets for name,t in self.progress.items()}})
            (output/'run_summary.json').write_text(json.dumps(summary,indent=2)+'\n')

    rclpy.init()
    monitor=PassiveMonitor()
    try:
        rclpy.spin(monitor)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        monitor.close()
        monitor.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        server.shutdown()
        server.server_close()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
