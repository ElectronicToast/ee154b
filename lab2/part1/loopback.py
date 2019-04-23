'''
This program sends byte characters over serial port to an Arduino,
then prints what the Arduino sends back.
'''

from time import sleep
import serial

BAUDRATE = 9600
PORT = 'COM10'

def main():
    # Open the serial port
    ser = serial.Serial(PORT, BAUDRATE)

    # Confirm that the connection is established
    if ser.isOpen():
        print('Serial port opened successfully.')

    # Continuously ask for input
    while True:
        i = input(">> ")
        if i == 'quit' or i == 'q' or i == 'Q':
            ser.close()
            exit()
        else:
            ser.write(bytes(i,'utf-8'))
            sleep(0.1)
            out = ''            

            # in case there is more than 1 byte waiting for us, read all of them
            while ser.in_waiting > 0:
                out += ser.read(1).decode('utf-8')

            # don't print empty reads
            if out != '':
                print(out)

if __name__ == '__main__':
    main()


