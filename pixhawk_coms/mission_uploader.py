import json
from pymavlink import mavutil, mavwp

pf_path = '../pathFinder/maps/path.plan'
sp_path = '../searchPattern/maps/searchPattern.plan'

with open(sp_path, 'r') as f:
    plan_data = json.load(f)

mission_items = plan_data['mission']['items']

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
master.wait_heartbeat()

master.mav.mission_clear_all_send(master.target_system, master.target_component)

wp = mavwp.MAVWPLoader()

for i, item in enumerate(mission_items):
    msg = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system, master.target_component,
        int(i),
        int(item['frame']),
        int(item['command']),
        0, 1,
        float(item['params'][0]), float(item['params'][1]),
        float(item['params'][2]), float(item['params'][3]),
        int(item['params'][4] * 1e7),
        int(item['params'][5] * 1e7),
        float(item['params'][6])
    )

    wp.add(msg)

master.waypoint_count_send(wp.count())

for i in range(wp.count()):
    msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=5)
    
    #if msg is None:
    #    print(f"Protocol Timeout at index {i}")
    #    break

    master.mav.send(wp.wp(msg.seq))
    print(f"Sending waypoint {msg.seq}")

ack = master.recv_match(type=['MISSION_ACK'], blocking=True)
print(f"Mission upload result: {ack}")
