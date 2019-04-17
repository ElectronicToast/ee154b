'''
This program sends byte characters over serial port to an Arduino,
then prints what the Arduino sends back.
'''

from time import sleep
import 

BAUDRATE = 9600
PORT = 'COM8'

def main():
    # Open the serial port
    ser = serial.Serial(PORT, BAUDRATE)

    # Confirm that the connection is established
    if ser.isOpen():
        print('Serial port opened successfully.')

    # Continuously ask for input
    while True:
    i = input(">> ")
    if i == 'quit':
        ser.close()
        exit()
    else:
        out = ''
        ser.write(bytes(i,'utf-8'))
        sleep(0.1)

    # in case there is more than 1 byte waiting for us, read all of them
    while ser.in_waiting > 0:
        out += str(ser.read(1))

    # don't print empty reads
    if out != '':
        print(out)

if __name__ == '__main__':
    main()


