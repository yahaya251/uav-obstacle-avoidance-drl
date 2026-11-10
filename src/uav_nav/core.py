"""ROS-independent observation, recommendation and monitoring contracts.

水平FLU扫描→世界ENU36扇区；range/goal/velocity form 40 values.
These development guards produce recommendations, not certified flight safety.
"""
from dataclasses import dataclass, asdict
import csv
import json
import math
from pathlib import Path
from typing import Optional

TAU = 2 * math.pi


@dataclass(frozen=True)
class Config:
    rays: int = 36
    sensor_range: float = 6.0
    goal_scale: float = 12.0
    max_speed: float = 1.0
    max_acceleration: float = 1.0
    max_scan_age: float = 0.5
    max_pose_age: float = 0.5
    max_pair_skew: float = 0.25
    max_tilt_rad: float = math.radians(15)
    vehicle_radius: float = 0.2
    stop_margin: float = 0.35
    goal_radius: float = 0.35
    positive_inf_is_clear: bool = True

    def __post_init__(self):
        if self.rays != 36:
            raise ValueError('The observation contract has 36 range bins.')
        positive = ('sensor_range','goal_scale','max_speed','max_acceleration',
                    'max_scan_age','max_pose_age','max_pair_skew','max_tilt_rad',
                    'vehicle_radius','stop_margin','goal_radius')
        if any(not math.isfinite(getattr(self, k)) or getattr(self, k) <= 0 for k in positive):
            raise ValueError('Configuration values must be positive and finite.')


@dataclass(frozen=True)
class PoseSample:
    position_enu: tuple
    velocity_enu: tuple
    yaw_enu: float
    received_at: float
    source_timestamp: Optional[float] = None
    roll_rad: float = 0.0
    pitch_rad: float = 0.0


@dataclass(frozen=True)
class ScanSample:
    ranges: tuple
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    received_at: float
    source_timestamp: Optional[float] = None


@dataclass(frozen=True)
class ObservationResult:
    values: tuple
    ready: bool
    reasons: tuple
    coverage: tuple
    min_range_m: float
    scan_age_s: Optional[float]
    pose_age_s: Optional[float]
    invalid_samples: int = 0
    sample_count: int = 0
    scan_source_timestamp: Optional[float] = None
    pose_source_timestamp: Optional[float] = None

    @property
    def coverage_fraction(self):
        return sum(self.coverage) / len(self.coverage)


@dataclass(frozen=True)
class ActionResult:
    velocity_enu: tuple
    velocity_ned: tuple
    reason: str
    limited: bool


def _finite_vector(values, size):
    try:
        return len(values) == size and all(math.isfinite(float(v)) for v in values)
    except (TypeError,ValueError):
        return False


def ned_to_enu(vector):
    if not _finite_vector(vector, 3):
        raise ValueError('A finite 3-vector is required.')
    n, e, d = map(float, vector)
    return (e, n, -d)


def enu_to_ned(vector):
    return ned_to_enu(vector)


def pose_from_px4(position_ned, velocity_ned, q_wxyz, received_at,
                  source_timestamp=None, *, pose_frame=1, velocity_frame=1):
    """PX4 Hamilton q(wxyz), FRD body → NED world. Reject other frame enums.

    NED/ENU origins must already coincide. No estimated-origin alignment occurs.
    接收时间使用 monotonic，同源时间戳仅保留，不能混为消息年龄。
    """
    if pose_frame != 1 or velocity_frame != 1:
        raise ValueError('Only explicit PX4 NED pose_frame=1 and velocity_frame=1 are supported.')
    if not _finite_vector(q_wxyz, 4) or not math.isfinite(received_at):
        raise ValueError('Invalid attitude or reception time.')
    norm = math.sqrt(sum(float(v)**2 for v in q_wxyz))
    if not 0.5 <= norm <= 1.5:
        raise ValueError('Invalid quaternion norm.')
    w,x,y,z = (float(v)/norm for v in q_wxyz)
    yaw_ned = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    yaw_enu = (math.pi/2-yaw_ned+math.pi) % TAU-math.pi
    return PoseSample(ned_to_enu(position_ned), ned_to_enu(velocity_ned),
                      yaw_enu, float(received_at), source_timestamp, roll, pitch)


def build_observation(scan, pose, goal_enu, *, now, cfg=None, sensor_yaw_offset=0.0):
    """Min-pool measured rays into nearest world-ENU angular sectors.

    Unobserved/invalid bins are zero, never free space. Any invalid ray makes
    its sector unknown even when other rays in that sector are valid. +Inf is
    accepted as no-return only when positive_inf_is_clear is enabled for the
    verified sensor. Finite ranges outside [range_min,range_max] are invalid.
    A scan with angular gaps wider than a sector is rejected. This is not
    deskewed scanning: moving/tilted sensor compensation remains future work.
    """
    c = cfg or Config()
    reasons = []
    ranges = [c.sensor_range]*c.rays
    covered, invalid_bins = [False]*c.rays, [False]*c.rays
    invalid = 0
    scan_age = pose_age = None
    if not math.isfinite(now):
        raise ValueError('now must use a finite monotonic reception clock.')
    if not _finite_vector(goal_enu, 2):
        raise ValueError('A finite ENU goal pair is required.')
    valid_pose = pose is not None
    if valid_pose:
        valid_pose = (_finite_vector(pose.position_enu,3) and _finite_vector(pose.velocity_enu,3)
                      and all(math.isfinite(v) for v in (pose.yaw_enu,pose.received_at,
                                                         pose.roll_rad,pose.pitch_rad)))
    if not valid_pose:
        reasons.append('missing_or_invalid_pose')
    else:
        pose_age = now-pose.received_at
        if pose_age < -1e-6 or pose_age > c.max_pose_age:
            reasons.append('stale_or_future_pose')
        if max(abs(pose.roll_rad),abs(pose.pitch_rad)) > c.max_tilt_rad:
            reasons.append('tilt_exceeds_planar_assumption')
        if math.hypot(*pose.velocity_enu[:2]) > c.max_speed+1e-6:
            reasons.append('measured_speed_exceeds_contract')
    scan_meta_ok = scan is not None
    if scan_meta_ok:
        scan_meta_ok = (bool(scan.ranges) and all(math.isfinite(v) for v in
                       (scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,scan.received_at))
                       and scan.angle_increment != 0 and 0 <= scan.range_min < scan.range_max)
    if not math.isfinite(sensor_yaw_offset):
        scan_meta_ok = False
    if not scan_meta_ok:
        reasons.append('missing_or_invalid_scan_metadata')
    else:
        scan_age = now-scan.received_at
        if scan_age < -1e-6 or scan_age > c.max_scan_age:
            reasons.append('stale_or_future_scan')
        if valid_pose and abs(scan.received_at-pose.received_at) > c.max_pair_skew:
            reasons.append('scan_pose_reception_skew')
        if abs(scan.angle_increment) > TAU/c.rays+1e-9:
            reasons.append('scan_angular_gaps')
        if scan.range_max+1e-6 < c.sensor_range:
            reasons.append('sensor_range_below_observation_contract')
        yaw = pose.yaw_enu+sensor_yaw_offset if valid_pose else 0.0
        for i, sample in enumerate(scan.ranges):
            angle = scan.angle_min+i*scan.angle_increment+yaw
            sector = int(math.floor(((angle+math.pi/c.rays) % TAU)*c.rays/TAU)) % c.rays
            try:
                sample = float(sample)
            except (ValueError,TypeError):
                sample = float('nan')
            valid = math.isfinite(sample) and scan.range_min <= sample <= scan.range_max
            clear = sample == math.inf and c.positive_inf_is_clear
            if valid or clear:
                covered[sector] = True
                ranges[sector] = min(ranges[sector],c.sensor_range if clear else sample)
            else:
                invalid += 1
                invalid_bins[sector] = True
        if invalid:
            reasons.append('invalid_range_samples')
    coverage = tuple(a and not b for a,b in zip(covered,invalid_bins))
    ranges = tuple(value if known else 0.0 for value,known in zip(ranges,coverage))
    if not all(coverage):
        reasons.append('incomplete_360_degree_coverage')
    goal = tuple(max(-1.,min(1.,(goal_enu[i]-pose.position_enu[i])/c.goal_scale))
                 if valid_pose else 0.0 for i in range(2))
    velocity = tuple(max(-1.,min(1.,pose.velocity_enu[i]/c.max_speed))
                     if valid_pose else 0.0 for i in range(2))
    values = tuple(v/c.sensor_range for v in ranges)+goal+velocity
    return ObservationResult(values,not reasons,tuple(dict.fromkeys(reasons)),coverage,
                             min(ranges),scan_age,pose_age,invalid,len(scan.ranges) if scan else 0,
                             scan.source_timestamp if scan else None,
                             pose.source_timestamp if pose else None)


def apf_velocity(observation, cfg=None):
    """Reactive baseline sees only the same 40D vector and its validity gate."""
    c = cfg or Config()
    if not observation.ready:
        return (0.,0.)
    values = observation.values
    if len(values) != 40 or not all(math.isfinite(v) for v in values):
        return (0.,0.)
    gx,gy = values[36]*c.goal_scale,values[37]*c.goal_scale
    distance = math.hypot(gx,gy)
    if distance <= c.goal_radius:
        return (0.,0.)
    fx,fy = gx/distance,gy/distance
    for i,value in enumerate(values[:36]):
        clearance = max(.05,value*c.sensor_range-c.vehicle_radius)
        if clearance < 1.5:
            strength = min(3.,.06*(1/clearance-1/1.5)/(clearance*clearance))
            angle = i*TAU/c.rays
            fx -= strength*math.cos(angle)
            fy -= strength*math.sin(angle)
    norm = max(1.,math.hypot(fx,fy))
    return (c.max_speed*fx/norm,c.max_speed*fy/norm)


def limit_velocity(requested, previous, dt, *, cfg=None, ready=True, min_range_m=math.inf):
    """Clip norm/acceleration; invalid, stale or near-obstacle input → immediate zero.

    The hard-stop recommendation bypasses acceleration smoothing. It does not
    guarantee a real aircraft can stop instantly and is never published here.
    """
    c = cfg or Config()
    reason = 'ok'
    if not ready:
        reason = 'observation_not_ready'
    elif not math.isfinite(dt) or dt <= 0:
        reason = 'invalid_dt'
    elif math.isnan(min_range_m) or min_range_m < 0:
        reason = 'invalid_minimum_range'
    elif min_range_m <= c.vehicle_radius+c.stop_margin:
        reason = 'minimum_range_stop'
    else:
        try:
            if not _finite_vector(requested,2) or not _finite_vector(previous,2):
                reason = 'invalid_action'
        except (TypeError,ValueError):
            reason = 'invalid_action'
    if reason != 'ok':
        return ActionResult((0.,0.),(0.,0.,0.),reason,True)
    clipped = tuple(max(-c.max_speed,min(c.max_speed,float(v))) for v in requested)
    norm = max(1.,math.hypot(*clipped)/c.max_speed)
    target = tuple(v/norm for v in clipped)
    prevnorm = max(1.,math.hypot(*previous)/c.max_speed)
    previous = tuple(v/prevnorm for v in previous)
    difference = tuple(a-b for a,b in zip(target,previous))
    scale = min(1.,c.max_acceleration*dt/max(1e-12,math.hypot(*difference)))
    velocity = tuple(previous[i]+scale*difference[i] for i in range(2))
    limited = any(abs(velocity[i]-float(requested[i])) > 1e-10 for i in range(2))
    return ActionResult(velocity,enu_to_ned((*velocity,0.)), 'limited' if limited else 'ok',limited)


class MonitorWriter:
    """CSV+JSONL log; explicit recommendation-only schema and finite JSON.

    Source timestamps retain producer clock; age is measured only from the
    subscriber's monotonic received_at clock. Header age / motion deskew and
    PX4 clock synchronization require later integration validation.
    """
    schema_version = 'uav_nav.monitor.v1'

    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True,exist_ok=True)
        self.count = 0
        self.ready_count = 0
        self._csv = (self.directory/'monitor.csv').open('w',newline='')
        self._json = (self.directory/'monitor.jsonl').open('w')
        self.fields = ['sample','monotonic_s','east_m','north_m','up_m','speed_m_s',
                       'scan_age_s','pose_age_s','coverage_fraction','ready','reasons',
                       'min_range_m','recommended_east_m_s','recommended_north_m_s',
                       'action_reason','flight_command_sent']
        self.writer = csv.DictWriter(self._csv,fieldnames=self.fields)
        self.writer.writeheader()

    def write(self, now, pose, observation, action):
        if not math.isfinite(now) or len(observation.values) != 40 or not all(math.isfinite(v) for v in observation.values):
            raise ValueError('Cannot log a nonfinite clock or invalid 40D observation.')
        if not all(0 <= v <= 1 for v in observation.values[:36]) or not all(-1 <= v <= 1 for v in observation.values[36:]):
            raise ValueError('Observation violates its declared bounds.')
        if not _finite_vector(action.velocity_enu,2):
            raise ValueError('Cannot log a nonfinite recommendation.')
        self.count += 1
        self.ready_count += int(observation.ready)
        p = tuple(float(v) if math.isfinite(float(v)) else None for v in pose.position_enu) if pose else (None,None,None)
        speed = math.hypot(*pose.velocity_enu[:2]) if pose and _finite_vector(pose.velocity_enu,3) else None
        row = dict(zip(self.fields,[self.count,now,*p,
                   speed,
                   observation.scan_age_s,observation.pose_age_s,observation.coverage_fraction,
                   observation.ready,';'.join(observation.reasons),observation.min_range_m,
                   *action.velocity_enu,action.reason,False]))
        self.writer.writerow(row)
        payload = {'schema_version':self.schema_version,**row,
                   'observation':list(observation.values),
                   'coverage':list(observation.coverage),
                   'scan_source_timestamp':observation.scan_source_timestamp,
                   'pose_source_timestamp':observation.pose_source_timestamp,
                   'action':asdict(action),
                   'scope':'recommendation_only_no_autopilot_publication'}
        self._json.write(json.dumps(payload,allow_nan=False)+'\n')
        self._csv.flush()
        self._json.flush()
        return payload

    def close(self):
        self._csv.close()
        self._json.close()
        summary = {'schema_version':self.schema_version,'samples':self.count,
                   'ready_samples':self.ready_count,'flight_command_sent':False,
                   'scope':'monitoring_and_recommendations_only'}
        (self.directory/'monitor_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        return summary

    def __enter__(self):
        return self

    def __exit__(self,*_):
        self.close()
