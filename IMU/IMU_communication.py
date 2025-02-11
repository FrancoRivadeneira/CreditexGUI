#!/usr/bin/env python3
import subprocess
import os

os.chdir("/home/tumi/imu_int/")

def run_ISCommunicationsExample():
    command = "./ISCommunicationsExample /dev/ttyACM0"
    try:
        # Run the command in the terminal
        subprocess.run(command, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running command: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    run_ISCommunicationsExample()

