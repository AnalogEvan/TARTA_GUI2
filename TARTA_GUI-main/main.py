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
# Spark Pins (GPIO)
ENABLE_PIN = 5  # Pin 5 seems to be an enable pin for the spark circuit
RELAY_PIN = 6
BOOST_PIN = 13
# Pump Address (I2C)
MCP4725_ADDRESS = 0x60
MAX_DAC_VALUE = 4095 # 12-bit DAC, 4095 is max output

# (RTC and Config functions remain the same)
I2C_BUS_NUM_FOR_RTC = 1
DS3321_ADDRESS = 0x68
def bcd_to_dec(bcd): return ((bcd >> 4) * 10) + (bcd & 0x0F)
# ... (rest of helper functions are unchanged)

# --- RPi Controller ---
class RPIController:
    def __init__(self):
        self.operation_thread = None
        self.stop_operation = threading.Event()
        self.gpio_h = None
        self.dac = None

        if GPIO_MODE:
            try:
                self.gpio_h = lgpio.gpiochip_open(0)
                # Claim all spark-related output pins
                lgpio.gpio_claim_output(self.gpio_h, ENABLE_PIN)
                lgpio.gpio_claim_output(self.gpio_h, RELAY_PIN)
                lgpio.gpio_claim_output(self.gpio_h, BOOST_PIN)
                # Set initial states to LOW
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
                self.dac.raw_value = 0 # Ensure pump is off on startup
                print(f"Pump DAC initialized at address {hex(MCP4725_ADDRESS)}.")
            except Exception as e:
                print(f"FATAL: Could not initialize DAC. Error: {e}")
                self.dac = None

    def set_pump(self, state):
        """Controls the pump via the MCP4725 DAC."""
        if self.dac:
            self.dac.raw_value = MAX_DAC_VALUE if state else 0
        print(f"Pump DAC set to {'ON' if state else 'OFF'}")

   def _execute_spark_sequence(self):
        """Executes the 4-second ON timing sequence from test.py."""
        if self.gpio_h:
            print("SPARK: Turning ON pins 5 (Enable) and 13 (Boost) for 4 seconds.")
            # Turn on enable and boost circuits as per test.py
            lgpio.gpio_write(self.gpio_h, ENABLE_PIN, 1)
            lgpio.gpio_write(self.gpio_h, BOOST_PIN, 1)
            
            # Wait for 4 seconds
            time.sleep(4)
            
            # Turn pins off
            lgpio.gpio_write(self.gpio_h, BOOST_PIN, 0)
            lgpio.gpio_write(self.gpio_h, ENABLE_PIN, 0)
            print("SPARK: Pins OFF.")
            # Note: The 2-second OFF period from test.py is partially handled by the 
            # 1-second delay between sparks in the main scanning loop.
        else:
            # Simulation for testing without hardware
            print("SIM: 'Spark' ON for 4s")
            time.sleep(4)
            print("SIM: 'Spark' OFF")


    def cleanup(self):
        print("Cleaning up hardware resources...")
        if self.dac:
            self.dac.raw_value = 0 # Turn pump off
            print("Pump DAC set to 0.")
        if self.gpio_h:
            # Ensure all pins are off before closing
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
                
                # --- PUMP AND SPARK (as per your request) ---
                print(f"Cycle {c}: Turning pump ON.")
                self.set_pump(True)
                
                for s in range(1, sparks + 1):
                    if self.stop_operation.is_set(): break
                    eel.update_ui(f'SPARK,{s}')()
                    self._execute_spark_sequence()
                    
                    elapsed_time = time.time() - start_time
                    time_left_ms = max(0, (total_duration_sec - elapsed_time) * 1000)
                    eel.update_ui(f'TIME_LEFT,{int(time_left_ms)}')()
                    time.sleep(1) # Small delay between sparks
                
                print(f"Cycle {c}: Turning pump OFF.")
                self.set_pump(False)
                
                # --- WAIT for the remainder of the cycle time ---
                print(f"Cycle {c}: Waiting for next cycle.")
                cycle_duration = total_duration_sec / cycles
                while time.time() - cycle_start_time < cycle_duration:
                    if self.stop_operation.is_set(): break
                    elapsed_time = time.time() - start_time
                    time_left_ms = max(0, (total_duration_sec - elapsed_time) * 1000)
                    eel.update_ui(f'TIME_LEFT,{int(time_left_ms)}')()
                    time.sleep(0.5)
        finally:
            self.set_pump(False) # Final safety check
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
            
            eel.update_ui(f"HOURLY_NEXT_EVENT,{spark_start_time.isoformat()}")()
            
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
            
            eel.update_ui(f"HOURLY_NEXT_EVENT,{next_hour_time.isoformat()}")()

            for s in range(1, sparks + 1):
                if self.stop_operation.is_set():
                    print("Hourly Monitoring aborted during sparking.")
                    return
                self._execute_spark_sequence()
                time.sleep(1)

                if pst.localize(get_rtc_datetime()) >= next_hour_time:
                    print("Warning: Sparking overran the hour.")
                    break
            
            eel.update_ui(f"HOURLY_MONITOR_STATUS,Waiting for next hour,Next pump at {next_hour_time.strftime('%H:%M')}")()
            eel.update_ui(f"HOURLY_NEXT_EVENT,{next_hour_time.isoformat()}")()
            while pst.localize(get_rtc_datetime()) < next_hour_time:
                if self.stop_operation.is_set():
                    print("Hourly Monitoring aborted during waiting.")
                    return
                time.sleep(0.5)

        print("Hourly Monitoring sequence stopped.")

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
    return get_rtc_datetime().strftime("%Y-%m-%d %H:%M:%S")

eel.init('web')
rpi_controller = RPIController()

@eel.expose
def close_app():
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
def is_rpi_ready(): return RPI_MODE and rpi_controller.gpio_h is not None and rpi_controller.dac_h is not None

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
