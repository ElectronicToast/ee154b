import serial, time

# set up the serial connection via USB with the same baud rate as the arduino
arduino = serial.Serial('/dev/cu.usbmodem14101', 115200)

# allow 1 second for the system to sleep
time.sleep(1)

resets = 0
delim = '\n'

keys = ['a', 'b', 'c']
commands = {
    'a' : "a. Get instrument parameter from instrument",
    'b' : "b. Set instrument parameter",
    'c' : "c. Get RSSI"
    }

print("a. Get instrument parameter from instrument")
print("b. Set instrument parameter")
print("c. Get RSSI")

while True:
    key = input("Please input a command: ")
    if (key in keys):
        arduino.write(bytes(key, 'ASCII'))  # write the command key to the serial port
        data = arduino.read_until(bytes(delim, 'ASCII'))   # waits for the delim to continue
        

        if (key == 'b'):
            param = input("Please input the desired parameter state: ") # prompt for an the instrument parameter
            arduino.write(bytes(param + delim, 'ASCII')) # send the parameter value
            data = arduino.read_until(bytes(delim, 'ASCII')) # waits for the delim to continue

        elif (key == 'a'):
            data = arduino.read_until(bytes(delim, 'ASCII'))
            if data:
                print('Telemetry received: ', data.decode('ASCII'))

        elif (key == 'c'):
            data = arduino.read_until(bytes(delim, 'ASCII'))
            if data:
                print('RSSI: ', data.decode('ASCII'))

    else:
        print("Invalid command. Please try again.")

