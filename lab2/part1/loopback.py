'''
This program sends byte characters over serial port COM7 to an Arduino,
then prints what the Arduino sends back.
'''

from time import sleep
import serial

# Open the serial port on COM7 (windows)
ser = serial.Serial('COM7', 9600)

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
      
      
      
      '''
    try:
        ser_bytes = ser.readline()
        decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        print(decoded_bytes)
    except:
        print("Keyboard Interrupt")
        break


for i in range(4):
  c = input('hello?')
  ser.write('1')
  time.sleep(0.1)
  print(ser.readline())
  time.sleep(0.1)
'''