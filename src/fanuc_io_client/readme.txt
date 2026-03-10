================================================================================ FANUC I/O Client (fanuc_io_client) ================================================================================ 
A lightweight, reusable ROS 2 Python package for interacting with FANUC robot digital and robot I/O.

This client acts as a streamlined wrapper around the official "fanuc_gpio_controller" services (GetBoolIO and SetBoolIO), allowing developers to easily read and write I/O states (DI, DO, RI, RO, F) over the Remote Motion Interface (RMI) without manually constructing service requests and spinning future objects.

PREREQUISITES
ROS 2: Humble or newer.

Dependencies: rclpy, fanuc_msgs

Hardware: A physical FANUC controller with the following software options installed:

J519 (Stream Motion)

R912 (Remote Motion)
(Note: The S636 External Control Package includes both of these.)

Driver: The official FANUC ROS 2 Driver must be actively running and connected to the physical hardware (use_mock:=false).

INSTALLATION
Clone or place this package into your ROS 2 workspace "src" directory.

Build the package using colcon:
cd ~/ws_fanuc
colcon build --packages-select fanuc_io_client --symlink-install

Source your workspace:
source install/setup.bash

API REFERENCE
Class: FanucIOClient

Initialization:
init(self, node_name='fanuc_io_client_node')
Initializes the ROS 2 node and blocks until the physical FANUC I/O services are available on the network.

Reading I/O:
read_io(self, io_type: str, index: int) -> bool or None
Reads the state of the specified port. Returns True (ON), False (OFF), or None if the RMI server rejects the read.

Writing I/O:
write_io(self, io_type: str, index: int, value: bool) -> bool
Writes a state to the specified port. Returns True upon a successful controller handshake, or False if rejected.

Supported "io_type" arguments:

'DO' (Digital Output)

'DI' (Digital Input - Read Only)

'RO' (Robot Output)

'RI' (Robot Input - Read Only)

'F'  (Flag)

USAGE & EXAMPLE CODE
The FanucIOClient is designed to be imported directly into your custom motion scripts or high-level application logic.

Below is an example of a standalone Python script demonstrating how to import the class, read a sensor, and actuate a tool.

--- START EXAMPLE (example_usage.py) ---
import rclpy
import sys
from fanuc_io_client import FanucIOClient

def main():
rclpy.init()

print("Initializing FANUC I/O Client...")
io_handler = FanucIOClient(node_name='my_application_logic')
print("Successfully connected to the RMI server.")

try:
    # Example 1: Read a Digital Input (e.g., a part-present sensor)
    sensor_index = 1
    is_part_present = io_handler.read_io('DI', sensor_index)
    
    if is_part_present is None:
        print(f"Error: Could not read DI[{sensor_index}]. Check your physical mapping.")
    else:
        print(f"Sensor DI[{sensor_index}] state: {'DETECTED' if is_part_present else 'CLEAR'}")

    # Example 2: Write to a Digital Output (e.g., actuate a gripper)
    gripper_do_index = 4
    print(f"Attempting to close gripper on DO[{gripper_do_index}]...")
    
    success = io_handler.write_io('DO', gripper_do_index, True)
    
    if success:
        print("Gripper actuation command accepted by controller.")
    else:
        print(f"Error: Controller rejected write command to DO[{gripper_do_index}].")

except KeyboardInterrupt:
    print("\nProcess interrupted by user.")
finally:
    io_handler.destroy_node()
    rclpy.shutdown()
if name == 'main':
main()

--- END EXAMPLE ---
TROUBLESHOOTING
If you encounter errors while using this client, refer to the common issues below:

Script hangs at "Waiting for FANUC I/O services..."
Ensure your physical driver is running. The fanuc_gpio_controller services do not spawn correctly if the driver cannot reach the robot, or if you are running the driver in simulation mode (use_mock:=true). The RMI socket requires physical hardware to initialize.

"RMI Error Code: 1" during read/write
The FANUC controller actively rejected the I/O request. This is almost always due to configuration on the Teach Pendant:

Unassigned Port: Check MENU -> I/O on the Teach Pendant. Ensure the index you are requesting is physically mapped or mapped to Rack 89 (Virtual Memory). Attempting to poll an UNASG (Unassigned) port will crash the RMI request.
-> To fix this: Go to MENU -> I/O -> TYPE -> Digital -> CONFIG. Add the desired DO registers to the active range (assigning a Rack, Slot, and Start Point), and then cycle power (reboot the controller) to apply the changes.

UOP Lockout: If writing to a DO fails, check your User Operator Panel signals (MENU -> I/O -> UOP). If the robot is in Remote mode but an external PLC currently "owns" the I/O range, RMI will be blocked from overwriting it. Ensure the "Enable" UOP signal is high.

================================================================================
