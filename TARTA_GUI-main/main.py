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
import traceback

# --- RPi specific imports ---
# For I2C DAC control (Pump)
try:
    import board
    import busio
    import adafruit_mcp4725
    DAC_MODE = True
    print("Adafruit libraries for DAC loaded.")
except (NotImplementedError, ImportError):
    print("WARNING: Adafruit libraries not found. DAC will be in simulation mode.")
    DAC_MODE = False

# For GPIO control (Sparks)
try:
    import lgpio
    GPIO_MODE = True
    print("lgpio library for GPIO loaded.")
except ImportError:
    print("WARNING: lgpio library not found. Sparks will be in simulation mode.")
    GPIO_MODE = False

RPI_MODE = DAC_MODE and GPIO_MODE

# --- Hardware Configuration ---
ENABLE_PIN = 5
RELAY_PIN = 6
BOOST_PIN = 13
MCP4725_ADDRESS = 0x60
MAX_DAC_VALUE = 4095
DS3231_ADDRESS = 0x68
I2C_BUS = 1

# --- Helper Functions ---

def load_config():
    """Loads config.json."""
    config_path = "config.json"
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            return json.load(f)
    else:
        print(f"Error: config.json not found. Please create one.")
        exit()

config = load_config()

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
        if h: lgpio.i2c_close(h)
        print(f"Error reading from RTC: {e}")
        return datetime.datetime.now()

def monitor_usb_drives():
    """Monitors for new USB drives and notifies the frontend."""
    print("USB monitor thread started.")
    known_mounts = {part.mountpoint for part in psutil.disk_partitions() if 'removable' in part.opts}
    while True:
        try:
            current_mounts = {part.mountpoint for part in psutil.disk_partitions(all=False) if 'removable' in part.opts and part.fstype}
            new_mounts = current_mounts - known_mounts
            for mount in new_mounts:
                print(f"New USB drive detected at: {mount}")
                eel.show_usb_prompt(mount)()
            known_mounts = current_mounts
            time.sleep(3)
        except Exception as e:
            print(f"Error in USB monitor loop: {e}")
            time.sleep(10)

@eel.expose
def copy_data_to_usb(mount_point):
    """Copies all files from the './output' directory to the specified USB drive."""
    source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output')
    dest_dir = os.path.join(mount_point, 'TARTA_OUTPUT')
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


# --- RPi Controller Class ---
class RPIController:
    def __init__(self):
        self.operation_thread = None
        self.stop_operation = threading.Event()
        self.gpio_h = None
        self.dac = None

        if GPIO_MODE:
            try:
                self.gpio_h = lgpio.gpiochip_open(0)
                lgpio.gpio_claim_output(self.gpio_h, ENABLE_PIN)
                lgpio.gpio_claim_output(self.gpio_h, RELAY_PIN)
                lgpio.gpio_claim_output(self.gpio_h, BOOST_PIN)
                lgpio.gpio_write(self.gpio_h, ENABLE_PIN, 0)
                lgpio.gpio_write(self.gpio_h, RELAY_PIN, 0)
                lgpio.gpio_write(self.gpio_h, BOOST_PIN, 0)
                print("Spark GPIO pins (5, 6, 13) initialized.")
            except Exception as e:
                print(f"FATAL: Could not initialize GPIO. Error: {e}")
                if self.gpio_h: lgpio.gpiochip_close(self.gpio_h)
                self.gpio_h = None
        
        if DAC_MODE:
            try:
                i2c_bus = busio.I2C(board.SCL, board.SDA)
                self.dac = adafruit_mcp4725.MCP4725(i2c_bus, address=MCP4725_ADDRESS)
                self.dac.raw_value = 0
                print(f"Pump DAC initialized at address {hex(MCP4725_ADDRESS)}.")
            except Exception as e:
                print(f"FATAL: Could not initialize DAC. Error: {e}")
                self.dac = None

    def set_pump(self, state):
        if self.dac:
            self.dac.raw_value = MAX_DAC_VALUE if state else 0
        print(f"Pump DAC set to {'ON' if state else 'OFF'}")

    def _execute_spark_sequence(self):
        if self.gpio_h:
            print("SPARK: Turning ON pins 5 (Enable) and 13 (Boost) for 4 seconds.")
            lgpio.gpio_write(self.gpio_h, ENABLE_PIN, 1)
            lgpio.gpio_write(self.gpio_h, BOOST_PIN, 1)
            time.sleep(4)
            lgpio.gpio_write(self.gpio_h, BOOST_PIN, 0)
            lgpio.gpio_write(self.gpio_h, ENABLE_PIN, 0)
            print("SPARK: Pins OFF.")
        else:
            print("SIM: 'Spark' ON for 4s")
            time.sleep(4)
            print("SIM: 'Spark' OFF")

    def cleanup(self):
        print("Cleaning up hardware resources...")
        if self.dac:
            self.dac.raw_value = 0
            print("Pump DAC set to 0.")
        if self.gpio_h:
            lgpio.gpio_write(self.gpio_h, ENABLE_PIN, 0)
            lgpio.gpio_write(self.gpio_h, RELAY_PIN, 0)
            lgpio.gpio_write(self.gpio_h, BOOST_PIN, 0)
            lgpio.gpiochip_close(self.gpio_h)
            print("GPIO resources released.")

    def run_scan_sequence(self, duration_min, sparks, cycles):
        print(f"Starting scan: {duration_min} mins, {sparks} sparks, {cycles} cycles")
        self.stop_operation.clear()
        try:
            total_duration_sec = duration_min * 60
            start_time = time.time()
            for c in range(1, cycles + 1):
                if self.stop_operation.is_set(): break
                eel.update_ui(f'CYCLE,{c}')()
                cycle_start_time = time.time()
                print(f"Cycle {c}: Turning pump ON.")
                self.set_pump(True)
                for s in range(1, sparks + 1):
                    if self.stop_operation.is_set(): break
                    eel.update_ui(f'SPARK,{s}')()
                    self._execute_spark_sequence()
                    elapsed_time = time.time() - start_time
                    time_left_ms = max(0, (total_duration_sec - elapsed_time) * 1000)
                    eel.update_ui(f'TIME_LEFT,{int(time_left_ms)}')()
                    time.sleep(1)
                print(f"Cycle {c}: Turning pump OFF.")
                self.set_pump(False)
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
                        self._execute_spark_sequence()
                        time.sleep(0.5)
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
                    time.sleep(1)
                    if pst.localize(get_rtc_datetime()) >= next_hour_time: break
                eel.update_ui(f"HOURLY_MONITOR_STATUS,Waiting for next hour,Next pump at {next_hour_time.strftime('%H:%M')}")()
                eel.update_ui(f"HOURLY_NEXT_EVENT,{next_hour_time.isoformat()}")()
                while pst.localize(get_rtc_datetime()) < next_hour_time:
                    if self.stop_operation.is_set(): return
                    time.sleep(0.5)
            except Exception as e:
                print(f"Error in hourly loop: {e}")
                time.sleep(300)
    
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

# --- Eel Exposed Functions ---
eel.init('web')
rpi_controller = RPIController()

@eel.expose
def get_rtc_time_str():
    return get_rtc_datetime().strftime("%Y-%m-%d %H:%M:%S")
@eel.expose
def close_app(): sys.exit(0)
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
def abort_all(): rpi_controller.abort_operation()
@eel.expose
def is_rpi_ready(): return RPI_MODE and rpi_controller.gpio_h is not None and rpi_controller.dac is not None
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
    combined_df = pd.concat([df.iloc[:min(len(df) for df in dfs)] for df in dfs])
    avg_df = combined_df.groupby(combined_df.columns[0])[combined_df.columns[1]].mean().reset_index()
    x, y = avg_df.iloc[:,0].tolist(), avg_df.iloc[:,1].tolist()
    peaks, _ = find_peaks(y, height=6500, distance=50)
    return {'x': x, 'y': y, 'peaks': peaks.tolist()}

# --- Main Execution Block ---
if __name__ == '__main__':
    try:
        usb_thread = threading.Thread(target=monitor_usb_drives)
        usb_thread.daemon = True
        usb_thread.start()
        eel.start('index.html', size=(1280, 800))
    except Exception as e:
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print(f"AN UNEXPECTED ERROR OCCURRED: {e}")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        traceback.print_exc()
    finally:
        print("UI closed, shutting down application.")
        if 'rpi_controller' in locals():
            rpi_controller.abort_operation()
            rpi_controller.cleanup()
        print("Application has been shut down.")
