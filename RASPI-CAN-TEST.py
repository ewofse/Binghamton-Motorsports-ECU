import can

# Initialize the CAN interface
bus = can.interface.Bus(channel='can0', bustype='socketcan')

try:
    print("CAN Bus Receiver Started...")

    # Continuous loop to receive CAN messages
    while True:
        # Receive a message from the CAN bus
        message = bus.recv()

        # Check if the message is from the Teensy (you might need to adjust the CAN ID)
        if message.arbitration_id == 0x201:  # Adjust the CAN ID based on your Teensy configuration
            data = message.data  # This contains the received data as bytes

            # Parse the received data
            reg_id = data[0]
            value = (data[1] << 8) | data[2]  # Combine the two bytes into a 16-bit value

            print("Received Data - Reg ID: {}, Value: {}".format(reg_id, value))

except KeyboardInterrupt:
    # Handle keyboard interrupt (Ctrl+C) to gracefully exit the program
    print("Keyboard Interrupt. Exiting...")
finally:
    # Close the CAN bus when done
    bus.shutdown()
    print("CAN Bus Receiver Stopped.")
