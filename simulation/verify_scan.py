#!/usr/bin/env python3
"""Read one complete ROS scan, without ros2 echo's array truncation."""
import json, math, time
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init();node=Node('uav_scan_acceptance'); result=[]
def callback(msg):
    ranges=list(msg.ranges)
    finite=[r for r in ranges if math.isfinite(r)]
    span=msg.angle_max-msg.angle_min
    row={'recorded_at':datetime.now().astimezone().isoformat(),
         'topic':'/uav/scan','source':'Gazebo gpu_lidar via ros_gz_bridge',
         'frame_id':msg.header.frame_id,'samples':len(ranges),
         'min_angle_rad':msg.angle_min,'max_angle_rad':msg.angle_max,
         'angle_increment_rad':msg.angle_increment,'span_degrees':span*180/math.pi,
         'range_min_m':msg.range_min,'range_max_m':msg.range_max,
         'finite_count':len(finite),'positive_inf_count':sum(r==math.inf for r in ranges),
         'nan_count':sum(math.isnan(r) for r in ranges),
         'minimum_finite_range_m':min(finite) if finite else None,
         'forward_sample_m':ranges[len(ranges)//2] if math.isfinite(ranges[len(ranges)//2]) else None,
         'source_timestamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
         'flight_command_sent':False}
    row['passed']=(len(ranges)==720 and span>=2*math.pi-1e-5 and bool(finite)
                   and row['positive_inf_count']>0 and row['nan_count']==0)
    result.append(row)
node.create_subscription(LaserScan,'/uav/scan',callback,qos_profile_sensor_data)
deadline=time.monotonic()+20
while not result and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
node.destroy_node();rclpy.shutdown()
if not result:raise SystemExit('No complete ROS LaserScan received within 20 seconds.')
print(json.dumps(result[0],indent=2,allow_nan=False))
raise SystemExit(0 if result[0]['passed'] else 1)
