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
import socket

# --- RPi specific imports ---
try:
    # lgpio is used for GPIO pins (relay, boost) and the RTC
    import lgpio 
    # Adafruit libraries are now used for the pump DAC
    import board
    import busio
    import adafruit_mcp4725
    # ntplib is for syncing time over the internet
    import ntplib
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
    """ Loads config.json. """
    config_path = "config.json"
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            try:
                return json.load(f)
            except json.JSONDecodeError:
                print("Warning: config.json is corrupted. Exiting.")
                sys.exit(1)
    else:
        print(f"Error: config.json not found. Please create one.")
        sys.exit(1)

config = load_config()

# --- RTC Helper Functions ---
def bcd_to_dec(bcd):
    """Convert Binary Coded Decimal to Decimal"""
    return (bcd // 16 * 10) + (bcd % 16)

def dec_to_bcd(dec):
    """Convert Decimal to Binary Coded Decimal"""
    return (dec // 10 * 16) + (dec % 10)

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
            return datetime.datetime(
                year=bcd_to_dec(time_data[6]) + 2000,
                month=bcd_to_dec(time_data[5] & 0x1F),
                day=bcd_to_dec(time_data[4]),
                hour=bcd_to_dec(time_data[2] & 0x3F),
                minute=bcd_to_dec(time_data[1]),
                second=bcd_to_dec(time_data[0] & 0x7F)
            )
        raise IOError(f"Expected 7 bytes from RTC, got {count}")
    except Exception as e:
        if h: lgpio.i2c_close(h)
        print(f"Error reading from RTC: {e}")
        return datetime.datetime.now()

def set_rtc_datetime(dt):
    """Writes a datetime object to the DS3231 RTC module."""
    if not RPI_MODE:
        print("Simulation mode: Cannot set RTC time.")
        return
    h = None
    try:
        time_data = [
            dec_to_bcd(dt.second), dec_to_bcd(dt.minute), dec_to_bcd(dt.hour),
            dec_to_bcd(dt.weekday() + 1), dec_to_bcd(dt.day),
            dec_to_bcd(dt.month), dec_to_bcd(dt.year - 2000)
        ]
        h = lgpio.i2c_open(I2C_BUS, DS3231_ADDRESS)
        lgpio.i2c_write_i2c_block_data(h, 0, time_data)
        lgpio.i2c_close(h)
        print(f"RTC time set to: {dt.strftime('%Y-%m-%d %H:%M:%S')}")
    except Exception as e:
        if h: lgpio.i2c_close(h)
        print(f"Error setting RTC time: {e}")

def sync_rtc_with_ntp():
    """Checks for internet and syncs RTC time with an NTP server."""
    if not RPI_MODE:
        return
    try:
        # Check for an internet connection by connecting to a known host
        socket.create_connection(("pool.ntp.org", 123), timeout=5)
        print("Internet connection detected. Attempting to sync RTC with NTP server...")
        client = ntplib.NTPClient()
        response = client.request('pool.ntp.org', version=3)
        ntp_time = datetime.datetime.fromtimestamp(response.tx_time)
        set_rtc_datetime(ntp_time)
    except (socket.gaierror, socket.timeout):
        print("No internet connection. Skipping RTC sync.")
    except Exception as e:
        print(f"An error occurred during NTP sync: {e}")

# --- USB Detection and Saving (Unchanged) ---
def monitor_usb_drives():
    # ... This function remains the same ...
    pass

@eel.expose
def copy_data_to_usb(mount_point):
    # ... This function remains the same ...
    pass

# --- RPi Controller ---
class RPIController:
    def __init__(self):
        self.operation_thread = None
        self.stop_operation = threading.Event()
        self.gpio_h = None
        self.dac = None

        if RPI_MODE:
            try:
                # Initialize GPIO for Relay and Boost using lgpio
                self.gpio_h = lgpio.gpiochip_open(0)
                lgpio.gpio_claim_output(self.gpio_h, RELAY_PIN)
                lgpio.gpio_claim_output(self.gpio_h, BOOST_PIN)
                self.set_relay(False)
                self.set_boost(False)
                print("GPIO pins initialized.")

                # Initialize DAC for Pump using Adafruit libraries
                i2c = busio.I2C(board.SCL, board.SDA)
                self.dac = adafruit_mcp4725.MCP4725(i2c, address=MCP4725_ADDRESS)
                self.set_pump(False)
                print("MCP4725 DAC for pump initialized successfully.")
            except Exception as e:
                print(f"FATAL: Could not initialize hardware. Error: {e}")
                self.cleanup()
                self.gpio_h, self.dac = None, None

    def set_pump(self, state):
        """Sets pump state using the Adafruit library."""
        if RPI_MODE and self.dac:
            self.dac.raw_value = 4095 if state else 0
        print(f"Pump DAC set to {'ON' if state else 'OFF'}.")

    def set_relay(self, state):
        """Controls the relay using lgpio."""
        if RPI_MODE and self.gpio_h:
            lgpio.gpio_write(self.gpio_h, RELAY_PIN, 1 if state else 0)
        print(f"Relay set to {'ON' if state else 'OFF'}")

    def set_boost(self, state):
        """Controls the boost pin using lgpio."""
        if RPI_MODE and self.gpio_h:
            lgpio.gpio_write(self.gpio_h, BOOST_PIN, 1 if state else 0)
        print(f"Boost set to {'ON' if state else 'OFF'}")

    def cleanup(self):
        if RPI_MODE:
            if self.dac: self.set_pump(False)
            if self.gpio_h:
                try: lgpio.gpiochip_close(self.gpio_h)
                except Exception as e: print(f"Error closing GPIO: {e}")
        print("Hardware cleanup complete.")

    def _execute_spark_sequence(self):
        """Executes one 4-second ON spark sequence."""
        self.set_boost(True)
        self.set_relay(True)
        time.sleep(4)
        self.set_relay(False)
        self.set_boost(False)

    def run_scan_sequence(self, duration_min, sparks, cycles):
        # ... This function remains the same ...
        pass

    def run_clean_sequence(self, sparks):
        # ... This function remains the same ...
        pass
        
    def run_pm_sequence(self, sparks, threshold, pm_type):
        # ... This function remains the same ...
        pass

    def run_hourly_monitoring_sequence(self):
        """Corrected hourly cycle: Pump -> Spark -> Wait."""
        print("Starting Corrected Hourly Monitoring sequence.")
        self.stop_operation.clear()
        pst = pytz.timezone('America/Los_Angeles')
        pump_duration_seconds = 120 # 2 minutes

        while not self.stop_operation.is_set():
            try:
                # --- 1. PUMPING STAGE ---
                print("HOURLY: Starting pump stage.")
                self.set_pump(True)
                eel.update_ui(f"HOURLY_MONITOR_STATUS,Pumping for {pump_duration_seconds / 60:.0f} mins,")()
                
                pump_start_time = time.time()
                while time.time() - pump_start_time < pump_duration_seconds:
                    if self.stop_operation.is_set():
                        self.set_pump(False)
                        print("Hourly Monitoring aborted during pumping.")
                        return
                    time.sleep(1)
                self.set_pump(False)
                print("HOURLY: Pumping complete.")

                # --- 2. SPARKING STAGE ---
                print("HOURLY: Starting spark stage.")
                now_naive = get_rtc_datetime()
                now_aware = pst.localize(now_naive, is_dst=None)
                sparks = 20 if now_aware.hour == 23 else 15
                
                eel.update_ui(f"HOURLY_MONITOR_STATUS,Sparking {sparks} times,")()
                for s in range(1, sparks + 1):
                    if self.stop_operation.is_set(): return
                    self._execute_spark_sequence()
                    time.sleep(2) # Wait between sparks
                print("HOURLY: Sparking complete.")

                # --- 3. WAITING STAGE ---
                print("HOURLY: Starting wait stage.")
                current_time = get_rtc_datetime()
                next_hour_time = (current_time + datetime.timedelta(hours=1)).replace(minute=0, second=0, microsecond=0)
                
                eel.update_ui(f"HOURLY_MONITOR_STATUS,Waiting,Next cycle at {pst.localize(next_hour_time).strftime('%H:%M')}")()
                eel.update_ui(f"HOURLY_NEXT_EVENT,{pst.localize(next_hour_time).isoformat()}")()
                
                while get_rtc_datetime() < next_hour_time:
                    if self.stop_operation.is_set():
                        print("Hourly Monitoring aborted during waiting.")
                        return
                    time.sleep(1)
                print("HOURLY: Wait complete. Starting new cycle.")

            except Exception as e:
                print(f"An error occurred in hourly monitor: {e}. Retrying in 5 mins.")
                time.sleep(300)

    def start_operation(self, target, *args):
        if self.operation_thread and self.operation_thread.is_alive(): return False
        self.stop_operation.clear()
        self.operation_thread = threading.Thread(target=target, args=args)
        self.operation_thread.daemon = True
        self.operation_thread.start()
        return True

    def abort_operation(self):
        if self.operation_thread and self.operation_thread.is_alive():
            self.stop_operation.set()
            eel.update_ui('STOPPED')()

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
    # Attempt to sync RTC with internet time at startup
    sync_rtc_with_ntp()
    
    eel.init('web')
    rpi_controller = RPIController()
    
    try:
        usb_thread = threading.Thread(target=monitor_usb_drives, daemon=True)
        usb_thread.start()
        eel.start('index.html', size=(1280, 800))
    except (SystemExit, MemoryError, KeyboardInterrupt):
        print("UI closed, shutting down application.")
    finally:
        rpi_controller.abort_operation()
        rpi_controller.cleanup()
        print("Application has been shut down.")

