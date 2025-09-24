# This script sets the time on the DS3231 Real-Time Clock (RTC).
# It can fetch the current time from the internet or set a time manually.
#
# USAGE:
#  - Automatic (Internet): sudo python3 set_time.py
#  - Manual: sudo python3 set_time.py --manual "YYYY-MM-DD HH:MM:SS"

import smbus2
import datetime
import time
import requests
import argparse

# I2C Configuration (must match main.py)
I2C_BUS = 1
DS3231_ADDRESS = 0x68

# --- Time fetching ---
def get_internet_time():
    """Fetches the current time from the World Time API for PST."""
    try:
        # Using a reliable public API to get the current time
        # This automatically handles DST for the specified timezone
        url = "http://worldtimeapi.org/api/timezone/America/Los_Angeles"
        response = requests.get(url, timeout=10)
        response.raise_for_status() # Raises an error for bad responses (4xx or 5xx)
        
        data = response.json()
        # The datetime is in ISO 8601 format, e.g., '2025-09-23T10:51:00.123456-07:00'
        # We parse it, ignoring the timezone info at the end as we just want wall-clock time
        iso_datetime = data['datetime']
        dt_object = datetime.datetime.fromisoformat(iso_datetime.split('.')[0])
        
        print(f"Successfully fetched internet time: {dt_object}")
        return dt_object
        
    except requests.exceptions.RequestException as e:
        print(f"Error: Could not connect to the internet to get time. {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred while fetching time: {e}")
        return None

# --- RTC Communication ---
def dec_to_bcd(dec):
    """Convert a decimal number to Binary Coded Decimal."""
    return (dec // 10 * 16) + (dec % 10)

def set_rtc_time(dt):
    """Sets the DS3231 RTC to the specified datetime object."""
    try:
        bus = smbus2.SMBus(I2C_BUS)
        
        time_data = [
            dec_to_bcd(dt.second),
            dec_to_bcd(dt.minute),
            dec_to_bcd(dt.hour),
            dec_to_bcd(dt.isoweekday()), # Monday=1, Sunday=7
            dec_to_bcd(dt.day),
            dec_to_bcd(dt.month),
            dec_to_bcd(dt.year - 2000)
        ]
        
        bus.write_i2c_block_data(DS3231_ADDRESS, 0x00, time_data)
        bus.close()
        print(f"Successfully set RTC time to: {dt.strftime('%Y-%m-%d %H:M:%S')}")

    except FileNotFoundError:
        print(f"Error: I2C bus {I2C_BUS} not found. Ensure I2C is enabled on the Pi.")
    except Exception as e:
        print(f"An error occurred while setting the RTC time: {e}")

def verify_rtc_time():
    """Reads the time back from the RTC to confirm it was set correctly."""
    def bcd_to_dec(bcd):
        return (bcd // 16 * 10) + (bcd % 16)
        
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
        
        read_back_time = datetime.datetime(year, month, date, hour, minute, sec)
        print(f"Verification: RTC time is now: {read_back_time.strftime('%Y-%m-%d %H:%M:%S')}")
    except Exception as e:
        print(f"Could not verify time after setting. Error: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set the DS3231 RTC time.")
    parser.add_argument(
        '-m', '--manual',
        type=str,
        help='Manually set the time using "YYYY-MM-DD HH:MM:SS" format.'
    )
    args = parser.parse_args()
    
    correct_time = None
    
    if args.manual:
        print("Manual mode selected.")
        try:
            correct_time = datetime.datetime.strptime(args.manual, '%Y-%m-%d %H:%M:%S')
        except ValueError:
            print('Error: Manual time format is incorrect. Please use "YYYY-MM-DD HH:MM:SS".')
            exit(1)
    else:
        print("Automatic mode selected. Attempting to fetch time from the internet...")
        correct_time = get_internet_time()

    if correct_time:
        set_rtc_time(correct_time)
        time.sleep(1) # Pause before verifying
        verify_rtc_time()
    else:
        print("Could not get a valid time. RTC was not updated.")
        exit(1)