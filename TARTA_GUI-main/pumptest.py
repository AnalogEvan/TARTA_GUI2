import time

# --- RPi specific imports ---
try:
    import lgpio
    RPI_MODE = True
    print("Running in Raspberry Pi mode using lgpio.")
except ImportError:
    print("FATAL: lgpio library not found. This script cannot run in simulation mode.")
    RPI_MODE = False
    exit()

# --- I2C Configuration ---
# These should match the values in your main application
I2C_BUS = 1
MCP4725_ADDRESS = 0x60 # Default I2C address for the MCP4725

def set_pump(i2c_handle, state):
    """
    Sets the MCP4725 DAC output.
    - True = On (max voltage, 4095)
    - False = Off (0 voltage, 0)
    """
    if not RPI_MODE or i2c_handle is None:
        print("Cannot set pump state (not in RPi mode or handle is invalid).")
        return

    try:
        # The MCP4725 is a 12-bit DAC, so the raw value ranges from 0 to 4095.
        value = 4095 if state else 0
        
        # The data for the MCP4725 needs to be sent as two bytes:
        # - The first byte contains the 8 most significant bits (MSB).
        # - The second byte contains the 4 least significant bits (LSB), shifted to the left by 4.
        data_to_send = [(value >> 4), (value & 0x0F) << 4]
        
        lgpio.i2c_write_device(i2c_handle, data_to_send)
        
        status = "ON (4095 -> ~5V)" if state else "OFF (0 -> 0V)"
        print(f"--> Pump command sent: {status}")

    except lgpio.error as e:
        print(f"An lgpio error occurred while writing to the DAC: {e}")
        print("Please check I2C wiring and ensure the address is correct.")

def main():
    """
    Main function to initialize I2C and provide a user interface for testing the pump.
    """
    if not RPI_MODE:
        return

    dac_handle = None
    try:
        # 1. Open a handle to the I2C device (the MCP4725 DAC)
        print(f"Attempting to open I2C device on bus {I2C_BUS} at address {hex(MCP4725_ADDRESS)}...")
        dac_handle = lgpio.i2c_open(I2C_BUS, MCP4725_ADDRESS)
        print("Successfully connected to I2C device.")
        
        # Ensure the pump is off at the start
        set_pump(dac_handle, False)
        print("\n--- Pump Test Utility ---")

        # 2. Enter a loop to get user commands
        while True:
            command = input("Enter 'on', 'off', or 'quit': ").lower()
            
            if command == "on":
                set_pump(dac_handle, True)
            elif command == "off":
                set_pump(dac_handle, False)
            elif command == "quit":
                print("Exiting test utility.")
                break
            else:
                print("Invalid command. Please try again.")

    except lgpio.error as e:
        print("-" * 50)
        print(f"FATAL ERROR: Could not open I2C device. {e}")
        print("Please check the following:")
        print("  1. The MCP4725 DAC is wired correctly (VCC, GND, SCL, SDA).")
        print(f"  2. The I2C address is correct (currently {hex(MCP4725_ADDRESS)}).")
        print("  3. I2C is enabled on your Raspberry Pi ('sudo raspi-config').")
        print("-" * 50)
        
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down.")

    finally:
        # 3. Clean up resources
        if dac_handle is not None:
            print("Cleaning up: Turning pump off and closing I2C handle.")
            # Ensure the pump is turned off before exiting
            set_pump(dac_handle, False)
            lgpio.i2c_close(dac_handle)
        print("Script finished.")

if __name__ == "__main__":
    main()
