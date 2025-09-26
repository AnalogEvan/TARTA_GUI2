import time
import board
import busio
import adafruit_mcp4725

# --- I2C Configuration ---
# This address should match your hardware and the other scripts.
I2C_BUS = 1 # This is handled by the 'board' library, but kept for reference
MCP4725_ADDRESS = 0x60 # Default I2C address for the MCP4725

# The MCP4725 is a 12-bit DAC, so the raw value ranges from 0 to 4095.
MAX_DAC_VALUE = 4095
MIN_DAC_VALUE = 0

def setup_dac():
    """Initializes the I2C bus and connects to the DAC using Adafruit libraries."""
    try:
        print("Initializing I2C bus...")
        # The 'board' library automatically finds the correct I2C pins (SCL, SDA)
        i2c = busio.I2C(board.SCL, board.SDA)
        print("I2C bus initialized.")

        print(f"Attempting to connect to DAC at address {hex(MCP4725_ADDRESS)}...")
        # Create a DAC instance using the Adafruit library.
        dac = adafruit_mcp4725.MCP4725(i2c, address=MCP4725_ADDRESS)
        print("SUCCESS: Successfully connected to MCP4725 DAC.")
        return dac
        
    except ValueError:
        print("-" * 50)
        print(f"FATAL ERROR: Could not find a device at I2C address {hex(MCP4725_ADDRESS)}.")
        print("Please double-check your wiring and the I2C address.")
        print("You can use 'sudo i2cdetect -y 1' in the terminal to scan for devices.")
        print("-" * 50)
        return None
    except Exception as e:
        print(f"An unexpected error occurred during setup: {e}")
        return None

def main():
    """
    Main function to provide a user interface for testing the pump.
    """
    # 1. Connect to the DAC
    dac = setup_dac()

    if dac is None:
        print("Exiting due to setup failure.")
        return

    # Ensure the pump is off at the start
    print("Setting initial pump state to OFF.")
    dac.raw_value = MIN_DAC_VALUE

    print("\n--- Pump Test Utility (Adafruit Library Version) ---")
    try:
        # 2. Enter a loop to get user commands
        while True:
            command = input("Enter 'on', 'off', or 'quit': ").lower()
            
            if command == "on":
                print(f"--> Sending ON command (raw value: {MAX_DAC_VALUE})")
                dac.raw_value = MAX_DAC_VALUE
                print("Pump should be ON.")
            elif command == "off":
                print(f"--> Sending OFF command (raw value: {MIN_DAC_VALUE})")
                dac.raw_value = MIN_DAC_VALUE
                print("Pump should be OFF.")
            elif command == "quit":
                print("Exiting test utility.")
                break
            else:
                print("Invalid command. Please try again.")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down.")
    except Exception as e:
        print(f"An error occurred during operation: {e}")
    finally:
        # 3. Clean up resources
        if dac:
            print("Cleaning up: Turning pump off.")
            dac.raw_value = MIN_DAC_VALUE
        print("Script finished.")

if __name__ == "__main__":
    main()

