from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)

print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received")

while True:
    try:
        msg = connection.recv_match(type='SYS_STATUS', blocking=True)
        voltage = msg.voltage_battery
        print(f"Battery voltage is {voltage} mv")
    except KeyboardInterrupt:
        break
