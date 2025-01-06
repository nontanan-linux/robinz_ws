import subprocess
import os

source_path = os.path.expanduser('~/.local/bin/auto_initialpose')
destination_path = os.path.expanduser('~/robinz_ws/install/robinz_vehicle_launch/lib/robinz_vehicle_launch/auto_initialpose')
password = os.getenv('SUDO_PASSWORD')

if not password:
    print("No password set in environment variable SUDO_PASSWORD")
    exit(1)

try:
    command = f'echo {password} | sudo -S mv {source_path} {destination_path}'
    subprocess.run(command, shell=True, check=True)
    print(f"Successfully moved {source_path} to {destination_path}")
except subprocess.CalledProcessError as e:
    print(f"Error occurred: {e}")
