import eel
import threading
import time
import glob
import os
import pandas as pd
import json
from scipy.signal import find_peaks
import numpy as np
import datetime
import pytz # Added for timezone support
import shutil
import psutil
import sys

# --- RPi specific imports ---
try:
    import RPi.GPIO as GPIO
    import smbus2
    import board
    import busio
    import adafruit_mcp4725
    RPI_MODE = True
    print("Running in Raspberry Pi mode.")
except ImportError:
    print("WARNING: RPi.GPIO, smbus2, or Adafruit libraries not found. Running in simulation mode.")
    RPI_MODE = False

# --- GPIO and I2C Configuration ---
# PUMP_PIN is no longer needed for direct control
RELAY_PIN = 6
BOOST_PIN = 13
I2C_BUS = 1
DS3231_ADDRESS = 0x68
MCP4725_ADDRESS = 0x62 # Default I2C address for the MCP4725

def load_config():
    """
    Loads config.json. If it doesn't exist, creates a default one.
    """
    config_path = "config.json"
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            try:
                config = json.load(f)
                print("Loading existing config.json.")
                return config
            except json.JSONDecodeError:
                print("Warning: config.json is corrupted. Exiting.")
                exit()
    else:
        print(f"Error: config.json not found. Please create one.")
        exit()

# Load configuration at startup
config = load_config()

# --- RTC Helper Functions ---
def bcd_to_dec(bcd):
    """Convert Binary Coded Decimal to Decimal"""
    return (bcd // 16 * 10) + (bcd % 16)

def get_rtc_datetime():
    """Reads the time from a DS3231 RTC module and returns a naive datetime object."""
    if not RPI_MODE:
        return datetime.datetime.now()
        
    try:
        bus = smbus2.SMBus(I2C_BUS)
        time_data = bus.read_i2c_block_data(DS3231_ADDRESS, 0, 7)
        bus.close()

        sec = bcd_to_dec(time_data[0] & 0x7F)
        minute = bcd_to_dec(time_data[1])
        hour = bcd_to_dec(time_data[2] & 0x3F)
        date = bcd_to_dec(time_data[4])
        month = bcd_to_dec(time_data[5] & 0x1F)
        year = bcd_to_dec(time_data[6]) + 2000
        
        return datetime.datetime(year, month, date, hour, minute, sec)

    except Exception as e:
        print(f"Error reading from RTC: {e}")
        # Fallback to system time if RTC fails
        return datetime.datetime.now()

# --- USB Detection and Saving ---
def monitor_usb_drives():
    """
    Monitors for new USB drives being connected and notifies the frontend.
    This is improved to filter out drives without a file system (like empty CD drives).
    """
    print("USB monitor thread started.")
    known_mounts = set()

    # Initial scan to populate known_mounts without triggering a popup on startup
    try:
        for part in psutil.disk_partitions():
            if 'removable' in part.opts and part.fstype:
                 known_mounts.add(part.mountpoint)
    except Exception as e:
        print(f"Initial USB scan failed: {e}")
    print(f"Known mounts at startup: {known_mounts}")

    while True:
        try:
            current_mounts = set()
            for part in psutil.disk_partitions(all=False): # all=False to ignore devices not ready
                 # Check for removable drives that HAVE a filesystem type.
                 # This helps filter out empty CD/DVD drives which have an empty fstype.
                 if 'removable' in part.opts and part.fstype:
                    current_mounts.add(part.mountpoint)
            
            new_mounts = current_mounts - known_mounts
            
            for mount in new_mounts:
                print(f"New valid USB drive detected at: {mount}")
                eel.show_usb_prompt(mount)()
            
            known_mounts = current_mounts
            time.sleep(3) # Check every 3 seconds
        except Exception as e:
            print(f"Error in USB monitor loop: {e}")
            time.sleep(10) # Wait longer if an error occurs

@eel.expose
def copy_data_to_usb(mount_point):
    """
    Copies all files from the local './output' directory to the specified USB drive.
    Includes a retry mechanism to handle drives that are not immediately ready.
    """
    source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output')
    dest_dir = os.path.join(mount_point, 'TARTA_OUTPUT')
    
    # --- Retry Loop ---
    # Wait for the drive to be fully ready before attempting to write.
    max_retries = 5
    retry_delay = 1 # in seconds
    drive_ready = False
    for attempt in range(max_retries):
        if os.path.exists(mount_point):
            drive_ready = True
            print(f"Drive {mount_point} is accessible. Proceeding with copy.")
            break
        else:
            print(f"Drive {mount_point} not ready, attempt {attempt + 1}/{max_retries}. Retrying in {retry_delay}s...")
            time.sleep(retry_delay)
            
    if not drive_ready:
        print(f"Error: Drive {mount_point} was not ready after {max_retries} attempts.")
        eel.usb_copy_status('error', 'Error: Drive was not ready.')()
        return

    if not os.path.isdir(source_dir):
        print(f"Source directory '{source_dir}' does not exist. Nothing to copy.")
        eel.usb_copy_status('error', 'Output directory not found.')()
        return

    try:
        print(f"Preparing to copy data to {dest_dir}")
        os.makedirs(dest_dir, exist_ok=True)
        
        files_to_copy = glob.glob(os.path.join(source_dir, '*'))
        if not files_to_copy:
            print("No files found in output directory to copy.")
            eel.usb_copy_status('success', 'No new files to copy.')()
            return
            
        copied_count = 0
        for file_path in files_to_copy:
            if os.path.isfile(file_path):
                print(f"Copying {file_path} to {dest_dir}")
                shutil.copy2(file_path, dest_dir)
                copied_count += 1
        
        print(f"Successfully copied {copied_count} files.")
        eel.usb_copy_status('success', f'Successfully copied {copied_count} files.')()

    except Exception as e:
        print(f"Error copying files to USB: {e}")
        eel.usb_copy_status('error', f'Error: Could not write to drive.')()
        
# --- RPi Controller ---
class RPIController:
    def __init__(self):
        self.operation_thread = None
        self.stop_operation = threading.Event()
        self.dac = None
        if RPI_MODE:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            # GPIO setup for pump is removed
            GPIO.setup(RELAY_PIN, GPIO.OUT)
            GPIO.setup(BOOST_PIN, GPIO.OUT)
            GPIO.output(RELAY_PIN, GPIO.LOW)
            GPIO.output(BOOST_PIN, GPIO.LOW)
            
            try:
                # Initialize I2C bus and MCP4725 DAC
                i2c = busio.I2C(board.SCL, board.SDA)
                self.dac = adafruit_mcp4725.MCP4725(i2c, address=MCP4725_ADDRESS)
                self.dac.normalized_value = 0.0 # Ensure pump is off at start
                print("GPIO pins and MCP4725 DAC initialized.")
            except Exception as e:
                print(f"FATAL: Could not initialize MCP4725 DAC. Error: {e}")
                # Potentially exit or handle the error appropriately
                self.dac = None


    def set_pump(self, state):
        # Control motor via DAC
        if RPI_MODE and self.dac:
            self.dac.normalized_value = 1.0 if state else 0.0
        print(f"Pump DAC set to {'1.0 (ON)' if state else '0.0 (OFF)'}")

    def set_relay(self, state):
        if RPI_MODE:
            GPIO.output(RELAY_PIN, GPIO.HIGH if state else GPIO.LOW)
        print(f"Relay set to {'ON' if state else 'OFF'}")

    def set_boost(self, state):
        if RPI_MODE:
            GPIO.output(BOOST_PIN, GPIO.HIGH if state else GPIO.LOW)
        print(f"Boost set to {'ON' if state else 'OFF'}")

    def cleanup(self):
        if RPI_MODE:
            # Set pump DAC to off before cleaning up GPIO
            if self.dac:
                self.dac.normalized_value = 0.0
            GPIO.cleanup()
        print("GPIO cleaned up and pump turned off.")

    # CHANGED: Consolidated spark logic into one method with test.py timing
    def _execute_spark_sequence(self):
        """Executes a single spark using the timing from test.py. WARNING: RISKS HARDWARE DAMAGE."""
        self.set_boost(True)
        self.set_relay(True)
        time.sleep(4)  # Hold components on for 4 seconds
        self.set_relay(False)
        self.set_boost(False)


    def run_scan_sequence(self, duration_min, sparks, cycles):
        print(f"Starting scan: {duration_min} mins, {sparks} sparks, {cycles} cycles")
        self.stop_operation.clear()
        
        try:
            self.set_pump(True)
            total_duration_sec = duration_min * 60
            start_time = time.time()

            for c in range(1, cycles + 1):
                if self.stop_operation.is_set(): break
                eel.update_ui(f'CYCLE,{c}')()
                
                cycle_start_time = time.time()
                
                for s in range(1, sparks + 1):
                    if self.stop_operation.is_set(): break
                    eel.update_ui(f'SPARK,{s}')()
                    
                    self._execute_spark_sequence() # CHANGED: Using the new spark method
                    
                    elapsed_time = time.time() - start_time
                    time_left_ms = max(0, (total_duration_sec - elapsed_time) * 1000)
                    eel.update_ui(f'TIME_LEFT,{int(time_left_ms)}')()
                    time.sleep(1) # Note: there is an additional 1-second wait here after the spark
                
                cycle_duration = total_duration_sec / cycles
                while time.time() - cycle_start_time < cycle_duration:
                    if self.stop_operation.is_set(): break
                    
                    elapsed_time = time.time() - start_time
                    time_left_ms = max(0, (total_duration_sec - elapsed_time) * 1000)
                    eel.update_ui(f'TIME_LEFT,{int(time_left_ms)}')()
                    time.sleep(0.5)
        finally:
            self.set_pump(False)
            eel.update_ui('DONE')()
            print("Scan sequence finished.")


    def run_clean_sequence(self, sparks):
        print(f"Starting clean: {sparks} sparks")
        self.stop_operation.clear()
        eel.update_ui('CYCLE,1')()

        for s in range(1, sparks + 1):
            if self.stop_operation.is_set(): break
            eel.update_ui(f'SPARK,{s}')()
            self._execute_spark_sequence() # CHANGED: Using the new spark method
            time.sleep(0.5)

        eel.update_ui('DONE')()
        print("Clean sequence finished.")
        
    def run_pm_sequence(self, sparks, threshold, pm_type):
        print(f"Starting PM monitoring: {sparks} sparks, threshold {threshold} for PM{pm_type}")
        self.stop_operation.clear()
        
        try:
            self.set_pump(True)
            while not self.stop_operation.is_set():
                current_pm_value = np.random.randint(0, int(threshold) + 20) 
                eel.update_ui(f'PM_VALUE,{current_pm_value}')()
                
                if current_pm_value >= threshold:
                    eel.update_ui('PM THRESHOLD REACHED')()
                    for s in range(1, sparks + 1):
                        if self.stop_operation.is_set(): break
                        eel.update_ui(f'SPARK,{s}')()
                        self._execute_spark_sequence() # CHANGED: Using the new spark method
                        time.sleep(0.5)
                    eel.update_ui('PM SPARKS COMPLETE')()
                
                time.sleep(2)
        finally:
            self.set_pump(False)
            print("PM monitoring stopped.")


    def run_hourly_monitoring_sequence(self):
        """Runs a continuous, time-based pumping and sparking cycle using the RTC."""
        print("Starting Hourly Monitoring sequence.")
        self.stop_operation.clear()
        pst = pytz.timezone('America/Los_Angeles')

        while not self.stop_operation.is_set():
            try:
                now_naive = get_rtc_datetime()
                now = pst.localize(now_naive, is_dst=None)
            except (pytz.AmbiguousTimeError, pytz.NonExistentTimeError) as e:
                print(f"Timezone localization error during DST change: {e}. Waiting 5 minutes.")
                time.sleep(300)
                continue

            spark_start_time = (now + datetime.timedelta(hours=1)).replace(minute=0, second=0, microsecond=0) - datetime.timedelta(minutes=2)
            if now >= spark_start_time:
                 spark_start_time += datetime.timedelta(hours=1)
            
            next_hour_time = (now + datetime.timedelta(hours=1)).replace(minute=0, second=0, microsecond=0)

            self.set_pump(True)
            eel.update_ui(f"HOURLY_MONITOR_STATUS,Pumping,Sparking at {spark_start_time.strftime('%H:%M')}")()
            
            while pst.localize(get_rtc_datetime()) < spark_start_time:
                if self.stop_operation.is_set():
                    self.set_pump(False)
                    print("Hourly Monitoring aborted during pumping.")
                    return
                time.sleep(1)
            
            self.set_pump(False)
            now = pst.localize(get_rtc_datetime())
            sparks = 20 if now.hour == 23 else 15
            
            eel.update_ui(f"HOURLY_MONITOR_STATUS,Sparking {sparks} times,Next pump at {next_hour_time.strftime('%H:%M')}")()

            for s in range(1, sparks + 1):
                if self.stop_operation.is_set():
                    print("Hourly Monitoring aborted during sparking.")
                    return
                self._execute_spark_sequence() # CHANGED: Using the new spark method
                time.sleep(1)

                if pst.localize(get_rtc_datetime()) >= next_hour_time:
                    print("Warning: Sparking overran the hour.")
                    break
            
            eel.update_ui(f"HOURLY_MONITOR_STATUS,Waiting for next hour,Next pump at {next_hour_time.strftime('%H:%M')}")()
            while pst.localize(get_rtc_datetime()) < next_hour_time:
                if self.stop_operation.is_set():
                    print("Hourly Monitoring aborted during waiting.")
                    return
                time.sleep(0.5)

        print("Hourly Monitoring sequence stopped.")

    # ... (the rest of the class methods start_operation and abort_operation remain the same) ...
    def start_operation(self, target, *args):
        if self.operation_thread and self.operation_thread.is_alive():
            print("Another operation is already in progress.")
            return False
        self.stop_operation.clear()
        self.operation_thread = threading.Thread(target=target, args=args)
        self.operation_thread.daemon = True
        self.operation_thread.start()
        return True

    def abort_operation(self):
        if self.operation_thread and self.operation_thread.is_alive():
            self.stop_operation.set()
            eel.update_ui('STOPPED')()
            print("Abort signal sent.")
        else:
            print("No operation to abort.")

@eel.expose
def get_rtc_time_str():
    """Exposes the RTC time as a formatted string to the frontend."""
    return get_rtc_datetime().strftime("%Y-%m-%d %H:%M:%S")

eel.init('web')
rpi_controller = RPIController()

@eel.expose
def close_app():
    """Exposed function to close the application from the frontend."""
    print("Close request received from frontend. Shutting down.")
    sys.exit(0)

@eel.expose
def get_config(): return config
@eel.expose
def start_scan(duration, sparks, cycles): return rpi_controller.start_operation(rpi_controller.run_scan_sequence, duration, sparks, cycles)
@eel.expose
def start_clean(sparks): return rpi_controller.start_operation(rpi_controller.run_clean_sequence, sparks)
@eel.expose
def start_pm(sparks, threshold, pm_type): return rpi_controller.start_operation(rpi_controller.run_pm_sequence, int(sparks), int(threshold), pm_type)
@eel.expose
def start_hourly_monitoring(): return rpi_controller.start_operation(rpi_controller.run_hourly_monitoring_sequence)
@eel.expose
def abort_all():
    rpi_controller.abort_operation()
    return True
@eel.expose
def is_rpi_ready(): return True

@eel.expose
def list_scans():
    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output')
    if not os.path.exists(output_path): os.makedirs(output_path)
    files = sorted(glob.glob(os.path.join(output_path, '*.csv')), key=os.path.getmtime, reverse=True)
    return [os.path.basename(f) for f in files]
@eel.expose
def get_scan_data(filename):
    full_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output', filename)
    if not os.path.exists(full_path): return {'x': [], 'y': [], 'peaks': []}
    df = pd.read_csv(full_path)
    x, y = df.iloc[:,0].tolist(), df.iloc[:,1].tolist()
    peaks, _ = find_peaks(y, height=6500, distance=50)
    return {'x': x, 'y': y, 'peaks': peaks.tolist()}
@eel.expose
def get_scan_data_avg():
    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output')
    files = sorted(glob.glob(os.path.join(output_path, '*.csv')), key=os.path.getmtime, reverse=True)[:10]
    if not files: return {'x': [], 'y': [], 'peaks': []}
    dfs = [pd.read_csv(f) for f in files]
    min_len = min(len(df) for df in dfs)
    dfs_trimmed = [df.iloc[:min_len, :] for df in dfs]
    combined_df = pd.concat(dfs_trimmed)
    avg_df = combined_df.groupby(combined_df.columns[0])[combined_df.columns[1]].mean().reset_index()
    x, y = avg_df.iloc[:,0].tolist(), avg_df.iloc[:,1].tolist()
    peaks, _ = find_peaks(y, height=6500, distance=50)
    return {'x': x, 'y': y, 'peaks': peaks.tolist()}

if __name__ == '__main__':
    try:
        # Start the USB monitoring thread
        usb_thread = threading.Thread(target=monitor_usb_drives)
        usb_thread.daemon = True
        usb_thread.start()
        
        eel.start('index.html', size=(1280, 800))
    except (SystemExit, MemoryError, KeyboardInterrupt):
        print("UI closed, shutting down application.")
    finally:
        rpi_controller.abort_operation()
        rpi_controller.cleanup()
        print("Application has been shut down.")

