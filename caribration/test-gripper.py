import time
from classrobot import gripper


_GRIPPER_LEFT_ = gripper.MyGripper3Finger()

# Initialize the gripper connection
host = "192.168.200.11"  # Replace with your gripper's IP address
port = 502              # Typically the default Modbus TCP port
print(f"Connecting to 3-Finger {host}:{port}", end="")

res = _GRIPPER_LEFT_.my_init(host=host, port=port)
if res:
    print("[SUCCESS]")
else:
    print("[FAILURE]")
    _GRIPPER_LEFT_.my_release()
    exit()

# Wait to ensure enough time has passed for the first command to execute
time.sleep(0.6)  # Delay slightly longer than the TIME_PROTECTION (0.5 s)

print("Testing gripper ...", end="")
_GRIPPER_LEFT_.my_hand_close()  # Now this should actuate the close command
time.sleep(2)                   # Wait for 2 seconds before the next command
# _GRIPPER_LEFT_.my_hand_open()    # Test open command
