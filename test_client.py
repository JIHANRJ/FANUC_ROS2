import rclpy
import sys
from fanuc_io_client import FanucIOClient

def main():
    # 1. Initialize ROS 2
    rclpy.init()

    print("\n" + "="*40)
    print(" FANUC I/O Client Test Script")
    print("="*40)

    # 2. Start the client (This will wait for the driver to be online)
    print("Starting client node...")
    io_client = FanucIOClient(node_name='test_io_client_node')
    
    # 3. Define what we are testing (using DO 4 based on your previous test)
    test_type = 'DO'
    test_index = 4

    print(f"\n--- Testing READ on {test_type}[{test_index}] ---")
    current_state = io_client.read_io(test_type, test_index)
    
    if current_state is not None:
        print(f"[SUCCESS] {test_type}[{test_index}] is currently: {'ON' if current_state else 'OFF'}")
    else:
        print(f"[FAILED] Could not read {test_type}[{test_index}].")
        print("-> Hint: Ensure the port is mapped (not UNASG) on the Teach Pendant.")

    print(f"\n--- Testing WRITE on {test_type}[{test_index}] ---")
    # We will try to write the opposite of whatever it currently is (or ON if the read failed)
    target_state = not current_state if current_state is not None else True
    print(f"Attempting to write: {'ON' if target_state else 'OFF'}")
    
    write_success = io_client.write_io(test_type, test_index, target_state)
    
    if write_success:
        print(f"[SUCCESS] Wrote to {test_type}[{test_index}] successfully.")
    else:
        print(f"[FAILED] Could not write to {test_type}[{test_index}].")
        print("-> Hint: Check if a PLC owns this port or if UOP signals are restricting access.")

    print("\n" + "="*40)
    print(" Test Finished")
    print("="*40 + "\n")

    # 4. Clean up and shutdown
    io_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
