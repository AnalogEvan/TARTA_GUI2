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
import pytz
import shutil
import psutil
import sys

# --- RPi specific imports ---
try:
    # lgpio is still used for GPIO pins (relay, boost) and the RTC
    import lgpio 
    # Adafruit libraries are now used for the pump DAC
    import board
    import busio
    import adafruit_mcp4725
    RPI_MODE = True
    print("Running in Raspberry Pi mode.")
except ImportError:
    print("WARNING: A required hardware library was not found. Running in simulation mode.")
    RPI_MODE = False

# --- GPIO and I2C Configuration ---
RELAY_PIN = 6 
BOOST_PIN = 13
I2C_BUS = 1
DS3231_ADDRESS = 0x68
MCP4725_ADDRESS = 0x60 # Default I2C address for the MCP4725

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

# --- RTC Helper Functions (using lgpio) ---
def bcd_to_dec(bcd):
    """Convert Binary Coded Decimal to Decimal"""
    return (bcd // 16 * 10) + (bcd % 16)

def get_rtc_datetime():
    """Reads the time from a DS3231 RTC module using lgpio."""
    if not RPI_MODE:
        return datetime.datetime.now()
    
    h = None
    try:
        h = lgpio.i2c_open(I2C_BUS, DS3231_ADDRESS)
        count, time_data = lgpio.i2c_read_i2c_block_data(h, 0, 7)
        lgpio.i2c_close(h)

        if count == 7:
            sec = bcd_to_dec(time_data[0] & 0x7F)
            minute = bcd_to_dec(time_data[1])
            hour = bcd_to_dec(time_data[2] & 0x3F)
            date = bcd_to_dec(time_data[4])
            month = bcd_to_dec(time_data[5] & 0x1F)
            year = bcd_to_dec(time_data[6]) + 2000
            
            return datetime.datetime(year, month, date, hour, minute, sec)
        else:
            raise IOError(f"Expected 7 bytes from RTC, got {count}")
    except Exception as e:
        if h is not None:
            lgpio.i2c_close(h)
        print(f"Error reading from RTC: {e}")
        return datetime.datetime.now()

# --- USB Detection and Saving ---
def monitor_usb_drives():
    print("USB monitor thread started.")
    known_mounts = set()
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
            for part in psutil.disk_partitions(all=False):
                if 'removable' in part.opts and part.fstype:
                    current_mounts.add(part.mountpoint)
            
            new_mounts = current_mounts - known_mounts
            for mount in new_mounts:
                print(f"New valid USB drive detected at: {mount}")
                eel.show_usb_prompt(mount)()
            
            known_mounts = current_mounts
            time.sleep(3)
        except Exception as e:
            print(f"Error in USB monitor loop: {e}")
            time.sleep(10)

@eel.expose
def copy_data_to_usb(mount_point):
    source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output')
    dest_dir = os.path.join(mount_point, 'TARTA_OUTPUT')
    
    # ... (rest of the function is unchanged)
    max_retries = 5
    retry_delay = 1
    drive_ready = False
    for attempt in range(max_retries):
        if os.path.exists(mount_point):
            drive_ready = True
            break
        else:
            time.sleep(retry_delay)
    if not drive_ready:
        eel.usb_copy_status('error', 'Error: Drive was not ready.')()
        return
    if not os.path.isdir(source_dir):
        eel.usb_copy_status('error', 'Output directory not found.')()
        return
    try:
        os.makedirs(dest_dir, exist_ok=True)
        files_to_copy = glob.glob(os.path.join(source_dir, '*'))
        if not files_to_copy:
            eel.usb_copy_status('success', 'No new files to copy.')()
            return
        copied_count = 0
        for file_path in files_to_copy:
            if os.path.isfile(file_path):
                shutil.copy2(file_path, dest_dir)
                copied_count += 1
        eel.usb_copy_status('success', f'Successfully copied {copied_count} files.')()
    except Exception as e:
        eel.usb_copy_status('error', f'Error: Could not write to drive.')()

# --- RPi Controller ---
class RPIController:
    def __init__(self):
        self.operation_thread = None
        self.stop_operation = threading.Event()
        self.gpio_h = None
        self.dac = None # MODIFIED: Changed from self.dac_h to self.dac

        if RPI_MODE:
            try:
                # --- Initialize GPIO for Relay and Boost using lgpio ---
                print("Initializing GPIO pins for relay and boost...")
                self.gpio_h = lgpio.gpiochip_open(0)
                lgpio.gpio_claim_output(self.gpio_h, RELAY_PIN)
                lgpio.gpio_claim_output(self.gpio_h, BOOST_PIN)
                lgpio.gpio_write(self.gpio_h, RELAY_PIN, 0)
                lgpio.gpio_write(self.gpio_h, BOOST_PIN, 0)
                print("GPIO pins initialized.")

                # --- Initialize DAC for Pump using Adafruit libraries ---
                print("Initializing I2C for DAC pump control...")
                i2c = busio.I2C(board.SCL, board.SDA)
                self.dac = adafruit_mcp4725.MCP4725(i2c, address=MCP4725_ADDRESS)
                self.set_pump(False) # Ensure pump is off on startup
                print("MCP4725 DAC for pump initialized successfully.")

            except Exception as e:
                print(f"FATAL: Could not initialize hardware. Error: {e}")
                self.cleanup()
                self.gpio_h = None
                self.dac = None

    def set_pump(self, state):
        """
        MODIFIED: Sets the MCP4725 DAC output using the Adafruit library.
        """
        if RPI_MODE and self.dac is not None:
            # Use the Adafruit library's 'raw_value' property.
            # 4095 is max voltage, 0 is zero voltage.
            self.dac.raw_value = 4095 if state else 0
        print(f"Pump DAC set to {'ON' if state else 'OFF'} using Adafruit library.")

    def set_relay(self, state):
        """ Unchanged: Controls the relay using lgpio. """
        if RPI_MODE and self.gpio_h is not None:
            lgpio.gpio_write(self.gpio_h, RELAY_PIN, 1 if state else 0)
        print(f"Relay set to {'ON' if state else 'OFF'}")

    def set_boost(self, state):
        """ Unchanged: Controls the boost pin using lgpio. """
        if RPI_MODE and self.gpio_h is not None:
            lgpio.gpio_write(self.gpio_h, BOOST_PIN, 1 if state else 0)
        print(f"Boost set to {'ON' if state else 'OFF'}")

    def cleanup(self):
        if RPI_MODE:
            # MODIFIED: Cleanup for the DAC
            if self.dac is not None:
                try:
                    self.set_pump(False)
                except Exception as e:
                    print(f"Error turning off pump during cleanup: {e}")
            
            # Unchanged: Cleanup for GPIO pins
            if self.gpio_h is not None:
                try:
                    # Turn pins off before closing
                    lgpio.gpio_write(self.gpio_h, RELAY_PIN, 0)
                    lgpio.gpio_write(self.gpio_h, BOOST_PIN, 0)
                    lgpio.gpiochip_close(self.gpio_h)
                except Exception as e:
                    print(f"Error closing GPIO chip handle: {e}")
        print("Hardware cleanup complete.")

    def _execute_spark_sequence(self):
        """
        This sequence now matches the timing from test.py.
        """
        print("Spark sequence: ON for 4 seconds.")
        self.set_boost(True)
        self.set_relay(True)
        time.sleep(4)
        self.set_relay(False)
        self.set_boost(False)

    # All the run_*_sequence methods below are UNCHANGED as they
    # correctly call the updated set_pump() and _execute_spark_sequence() methods.
    
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
                    self._execute_spark_sequence()
                    
                    elapsed_time = time.time() - start_time
                    time_left_ms = max(0, (total_duration_sec - elapsed_time) * 1000)
                    eel.update_ui(f'TIME_LEFT,{int(time_left_ms)}')()
                    time.sleep(2)
                
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
            self._execute_spark_sequence()
            time.sleep(2)

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
                        self._execute_spark_sequence()
                        time.sleep(2)
                    eel.update_ui('PM SPARKS COMPLETE')()
                time.sleep(2)
        finally:
            self.set_pump(False)
            print("PM monitoring stopped.")

    def run_hourly_monitoring_sequence(self):
        print("Starting Hourly Monitoring sequence.")
        self.stop_operation.clear()
        pst = pytz.timezone('America/Los_Angeles')

        while not self.stop_operation.is_set():
            try:
                now_naive = get_rtc_datetime()
                now = pst.localize(now_naive, is_dst=None)
            except (pytz.AmbiguousTimeError, pytz.NonExistentTimeError) as e:
                print(f"Timezone localization error: {e}. Waiting 5 minutes.")
                time.sleep(300)
                continue

            spark_start_time = (now + datetime.timedelta(hours=1)).replace(minute=0, second=0, microsecond=0) - datetime.timedelta(minutes=2)
            if now >= spark_start_time:
                spark_start_time += datetime.timedelta(hours=1)
            
            next_hour_time = (now + datetime.timedelta(hours=1)).replace(minute=0, second=0, microsecond=0)

            self.set_pump(True)
            eel.update_ui(f"HOURLY_MONITOR_STATUS,Pumping,Sparking at {spark_start_time.strftime('%H:%M')}")()
            eel.update_ui(f"HOURLY_NEXT_EVENT,{spark_start_time.isoformat()}")()
            
            while pst.localize(get_rtc_datetime()) < spark_start_time:
                if self.stop_operation.is_set():
                    self.set_pump(False)
                    return
                time.sleep(1)
            
            self.set_pump(False)
            now = pst.localize(get_rtc_datetime())
            sparks = 20 if now.hour == 23 else 15
            
            eel.update_ui(f"HOURLY_MONITOR_STATUS,Sparking {sparks} times,Next pump at {next_hour_time.strftime('%H:%M')}")()
            eel.update_ui(f"HOURLY_NEXT_EVENT,{next_hour_time.isoformat()}")()

            for s in range(1, sparks + 1):
                if self.stop_operation.is_set(): return
                self._execute_spark_sequence()
                time.sleep(2)
                if pst.localize(get_rtc_datetime()) >= next_hour_time:
                    break
            
            eel.update_ui(f"HOURLY_MONITOR_STATUS,Waiting for next hour,Next pump at {next_hour_time.strftime('%H:%M')}")()
            eel.update_ui(f"HOURLY_NEXT_EVENT,{next_hour_time.isoformat()}")()
            while pst.localize(get_rtc_datetime()) < next_hour_time:
                if self.stop_operation.is_set(): return
                time.sleep(0.5)

    def start_operation(self, target, *args):
        if self.operation_thread and self.operation_thread.is_alive():
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

@eel.expose
def get_rtc_time_str():
    return get_rtc_datetime().strftime("%Y-%m-%d %H:%M:%S")

eel.init('web')
rpi_controller = RPIController()

@eel.expose
def close_app():
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
def is_rpi_ready(): return RPI_MODE and rpi_controller.gpio_h is not None and rpi_controller.dac is not None

# The data handling functions below are unchanged.
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
