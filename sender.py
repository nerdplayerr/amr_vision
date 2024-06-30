import serial
import data_parser

# Define the serial port and baudrate
serial_port = '/dev/ttyUSB0'  # or 'COM1' for Windows
baud_rate = 115200

# Create a serial object
ser = serial.Serial(serial_port,
                     baudrate=baud_rate,
                     timeout=5.0,
                     bytesize=8,
                     parity='N',
                     stopbits=1)

while True:
        data_parser.send_command(ser,200,200,200)
        