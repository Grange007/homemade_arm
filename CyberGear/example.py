import serial
import time

ser = serial.Serial('/dev/ttyACM0', 921600, timeout=1)

# ser.write(bytes.fromhex('41 54 2b 41 54 0d 0a'))

# Rotate the motor forward for 1 second
ser.write(bytes.fromhex('41 54 90 07 e8 0c 08 05 70 00 00 07 01 95 54 0d 0a'))
time.sleep(1)

# Stop the motor for 1 second
ser.write(bytes.fromhex('41 54 90 07 e8 0c 08 05 70 00 00 07 00 7f ff 0d 0a'))
time.sleep(1)

# Rotate the motor backward for 1 second
ser.write(bytes.fromhex('41 54 90 07 e8 0c 08 05 70 00 00 07 01 6a aa 0d 0a'))
time.sleep(1)

# Stop the motor for 1 second
ser.write(bytes.fromhex('41 54 90 07 e8 0c 08 05 70 00 00 07 00 7f ff 0d 0a'))
time.sleep(1)

# Close the serial port
ser.close()