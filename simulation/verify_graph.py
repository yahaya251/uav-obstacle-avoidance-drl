#!/usr/bin/env python3
"""Read-only ROS graph acceptance for the application's passive interface."""
import json,time
from datetime import datetime
import rclpy
from rclpy.node import Node
rclpy.init();node=Node('uav_graph_acceptance')
deadline=time.monotonic()+8
subscriptions=[]
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.2)
    if ('uav_passive_monitor','/') not in node.get_node_names_and_namespaces():continue
    subscriptions=node.get_subscriber_names_and_types_by_node('uav_passive_monitor','/')
    if len(subscriptions)>=3:break
publishers=node.get_publisher_names_and_types_by_node('uav_passive_monitor','/')
allowed={'/rosout','/parameter_events'}
application=[name for name,types in publishers if name not in allowed]
expected={'/uav/scan','/fmu/out/vehicle_odometry','/fmu/out/vehicle_status_v1'}
report={'recorded_at':datetime.now().astimezone().isoformat(),'node':'/uav_passive_monitor',
        'publishers':publishers,'subscriptions':subscriptions,
        'explicit_application_publishers':application,
        'passed':not application and expected.issubset({name for name,types in subscriptions}),
        'scope':'Observed ROS graph; standard metadata publishers are allowed. No flight inputs published.'}
node.destroy_node();rclpy.shutdown()
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
