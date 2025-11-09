#!/usr/bin/python
import sys
import time
import tty          # Used for reading single keystrokes
import termios      # Used for reading single keystrokes
import select       # Used for non-blocking key reads
import csv          # Added for CSV logging
import datetime     # Added for unique log filenames

# --- REMEMBER TO CHANGE THIS ---
# Change 'amd64' to 'arm64' if you are on an ARM machine (like a Raspberry Pi)
sys.path.append('../lib/python/arm64')
import robot_interface as sdk

# --- Global Settings ---
HIGHLEVEL = 0xee
# --- CHECK YOUR IP AND PORTS ---
# ROBOT_IP = "192.168.123.161"  # Network cable
ROBOT_IP = "192.168.12.1"   # Wi-Fi
ROBOT_PORT = 8082
LOCAL_PORT = 8080

# --- Key checking function (non-blocking) ---
def get_key():
    """Checks for a key press and returns it, or None if no key is pressed."""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def print_instructions(mode, target_vel):
    """Prints the current state and controls."""
    mode_map = {
        0: "Idle",
        2: "Walk",
        7: "Damping",
        6: "Stand UP"
    }
    mode_str = mode_map.get(mode, f"Unknown ({mode})")
    
    print("\r" + " " * 80, end="") # Clear the line
    controls = "[W] Fwd | [S] Stop | [X] Back | [G] Damping | [Q] Quit | [K] Turn 90 degree ROLL | [H] Stand Up"
    status = f"Mode: {mode_str} | Vel: {target_vel:.1f}"
    print(f"\r{controls} || {status}", end="")

if __name__ == '__main__':
    
    # --- SAFETY WARNING ---
    print("---------------------------------------------------------------")
    print("WARNING: This script will make the robot move.")
    print("Please ensure the robot is in a clear, safe area.")
    print("Press Enter to continue...")
    input()
    print("Starting... Press 'Q' to exit.")
    
    # Save old terminal settings
    old_settings = termios.tcgetattr(sys.stdin)

    # --- LOGGING SETUP ---
    # Create a unique filename based on the current time
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"imu_log_{timestamp_str}.csv"
    print(f"Logging IMU data to: {log_filename}")

    try:
        # Open CSV file for writing
        # newline='' is recommended for the csv module to handle line endings correctly
        log_file = open(log_filename, 'w', newline='')
        csv_writer = csv.writer(log_file)
        # Write the Header row
        csv_writer.writerow(['timestamp', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'r', 'p', 'y'])

        # Set terminal to "cbreak" mode to read keys instantly
        tty.setcbreak(sys.stdin.fileno())

        # Initialize SDK objects
        udp = sdk.UDP(HIGHLEVEL, LOCAL_PORT, ROBOT_IP, ROBOT_PORT)
        cmd = sdk.HighCmd()
        state = sdk.HighState()
        udp.InitCmdData(cmd)

        current_velocity_x = 0.0
        currentYawSpeed = 0.0
        current_mode = 0 # Start in Idle mode
        print_instructions(current_mode, current_velocity_x)

        while True:
            # Main control loop at approx 50Hz
            time.sleep(0.02) 

            udp.Recv()
            udp.GetRecv(state)

            # --- CONTINUOUS LOGGING ---
            # Log data every loop iteration regardless of keypresses
            current_time = time.time()
            acc = state.imu.accelerometer
            gyro = state.imu.gyroscope
            rpy = state.imu.rpy
            
            # Ensure we actually have data before writing (sometimes init can be slow)
            if acc and gyro:
                 csv_writer.writerow([
                     current_time, 
                     acc[0], acc[1], acc[2], 
                     gyro[0], gyro[1], gyro[2],
                     rpy[0], rpy[1], rpy[2]
                 ])

            # Check for key presses
            key = get_key()
            if key:
                if key == 'w' or key == 'W':
                    current_mode = 2 # Walk mode
                    current_velocity_x = 0.4
                elif key == 's' or key == 'S':
                    current_mode = 2 # Stay in Walk mode, but stop moving
                    current_velocity_x = 0.0
                    currentYawSpeed = 0.0
                elif key == 'x' or key == 'X':
                    current_mode = 2 # Walk mode
                    current_velocity_x = -0.4
                elif key == 'g' or key == 'G':
                    current_mode = 7 # Damping mode
                    current_velocity_x = 0.0 
                elif key == 'h' or key == 'H':
                    current_mode = 6  # Stand Up 
                elif key == 'k' or key == 'K': 
                    current_mode = 2    # Turning Mode
                    current_velocity_x = 0.0
                    currentYawSpeed = 0.3
                
                
                elif key == 'q' or key == 'Q':
                    print("\nQuitting and setting robot to Idle...")
                    cmd.mode = 0
                    cmd.velocity = [0, 0]
                    udp.SetSend(cmd)
                    udp.Send()
                    break # Exit the while loop
                
                # Update display only on keypress to reduce terminal clutter
                print_instructions(current_mode, current_velocity_x)

            # Set the command packet
            cmd.mode = current_mode
            cmd.gaitType = 1            # 1 = trot gait
            cmd.velocity = [current_velocity_x, 0] 
            cmd.yawSpeed = currentYawSpeed      
            cmd.bodyHeight = 0.0        
            cmd.position = [0, 0]
            cmd.euler = [0.75, 0, 0]
            
            # Send the command
            udp.SetSend(cmd)
            udp.Send()

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        # --- CRITICAL RESTORE ---
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # Close the log file if it was opened
        try:
            log_file.close()
            print(f"\nLog file {log_filename} closed successfully.")
        except:
            pass