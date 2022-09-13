import robotiq_gripper
import time

# ip = "127.0.0.1"
ip="192.168.50.139"
def log_info(gripper):
    print(f"Pos: {str(gripper.get_current_position()): >3}  "
          f"Open: {gripper.is_open(): <2}  "
          f"Closed: {gripper.is_closed(): <2}  ")

print("Creating gripper...")
gripper = robotiq_gripper.RobotiqGripper()
print("Connecting to gripper...")
gripper.connect(ip, 63352)
print("Resting gripper...")
gripper._reset()
print("Activating gripper...")
gripper.activate()
time.sleep(1.5)

print("Testing gripper......")
gripper.move_and_wait_for_pos(255, 255, 255)
log_info(gripper)
print("gripper had closed!")
time.sleep(1.5)
gripper.move_and_wait_for_pos(0, 255, 255)
log_info(gripper)
print("gripper had opened!")
time.sleep(1.5)
