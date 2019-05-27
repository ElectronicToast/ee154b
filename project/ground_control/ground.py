from time import sleep, time
import serial, random, string, sys, logging, argparse, readline, coloredlogs
from datetime import datetime
import completer as completer
import kbhit as kbhit
import serial.tools.list_ports

# Constants
DEFAULT_BAUD_RATE = 9600
DEFAULT_SERIAL_PORT = 'DEF' # on windows
DEFAULT_LOGGING_LEVEL = 'DEBUG'
COMMAND_BOM_CHAR = '$' # beginning of message character for commands
TELEM_BOM_CHAR = '#' # beginning of message character for received telemetry
EOM_CHAR = ';' # marks end of time temperature data
DELIM = ',' # delimiter between commands and arguments
HEARTBEAT_INTERVAL = 60 # time in seconds between heartbeats
HEARTBEAT_TIMEOUT = 2 # time in seconds to wait for heartbeat response
HEARTBEAT_MSG = '$STAT,;'
RADIO_COMMAND_TIMEOUT = 2 # time to wait for radio command response

# Serial instance
ser = None
# Logger instance
logger = None
# time of last heartbeat sent
last_sent_heartbeat = None
# time of last transmission reponsded to
last_rec = None

PAYLOAD_MENU_ITEMS = [
    ('PWR', ['ON', 'OFF'], 'Turn the instrument on or off.'),
    ('PULS', ['0-100'], 'Instrument transmission duty-cycle.'),
    ('DATA', ['2400', '4800', '9600'], 'Set the instrument telemetry data rate.'),
    ('VOLT', [], 'Request instrument voltage.'),
    ('PRES', [], 'Request instrument pressure.'),
    ('STAT', [], 'Request instrument state.'),
    ('MOTR', ['0-100'], 'Set instrument flywheel speed.'),
    ('TEMP', [], 'Request instrument temperature.')]

BOX_MENU_ITEMS = [
    ('DOOR', [], 'Force open the side panel.'),
    ('PASS', ['anything'], 'Pass through any data.'),
    ('KP', ['int'], 'Change the Proportional constant for temperature PID loop.'),
    ('KI', ['int'], 'Change the Integral constant for temperature PID loop.'),
    ('KD', ['int'], 'Change the Derivative constant for temperature PID loop.'),
    ('BAUD', ['2400', '4800', '9600'], 'Set Arduino\'s data rate with LKM'),
    ('EMERG_KILL1', [], 'Initialize kill procedure.'),
    ('EMERG_KILL2', [], 'Confirm kill procedure.'),
    ('END_KILL', [], 'End kill.'),
    ('LKM_POWERON', ['2400', '4800', '9600'], 'Powers on LKM.'),
    ('ARDUINO_BAUD', ['2400', '4800', '9600'], 'Configures Arduino\'s baud rate with LKM.'),
    ('SYSTEM_BAUD', ['2400', '4800', '9600'], 'Changes system\'s baud rate.'),
    ('TARG', ['target temperature'], 'Changes the target control temperature.')]
    # probably some more like battery temp

ADMIN_MENU_ITEMS = [
    ('M', [], 'Print this menu.'),
    ('X', [], 'Quit command mode and return to LISTENING MODE.'),
    ('Q', [], 'Quit this ground command module.')]

def main(args):
    # build tab completer
    build_completer()

    # Configure the logger
    config_logger(args)

    # setup serial port
    setup_serial(args)

    # configure radio
    # config_radio()

    # print menu once
    print_menu()

    # listener for keyboard hits
    global kb
    kb = kbhit.KBHit()

    # send first heartbeat
    global last_sent_heartbeat
    last_sent_heartbeat = datetime.now()
    send_heartbeat()

    # start listening
    listening_state()

# $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ STATES $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
# state that constantly listens for transmissions and handles heartbeats and
# entering the command state
def listening_state():
    global last_sent_heartbeat
    just_entered = True
    while True:
        # check if need to print enter listening mode 
        if just_entered:
            just_entered = False
            logger.info('Listening...  press <c> to enter COMMAND MODE or <q> to quit program')
            sys.stdout.flush()
            # check if received anything when in command mode
            if ser.in_waiting > 0:
                logger.warning('\t RX (delayed): ' + serial_read())

        # check if receiving data
        if ser.in_waiting > 0:
            logger.warning('\t RX: ' + serial_read())
            sys.stdout.flush()

        # check if key is pressed
        if kb.kbhit():
            c = kb.getch()
            if c == 'c' or c == 'C':
                command_state()
                just_entered = True
            elif c == 'q' or c == 'Q':
                logger.info('Program terminated.')
                sys.exit(0)

        # send heartbeat
        if (datetime.now() - last_sent_heartbeat).seconds > HEARTBEAT_INTERVAL:
            last_sent_heartbeat = datetime.now()
            send_heartbeat()

# state that prompts for command input
def command_state():
    logger.info('Entered COMMAND MODE.')
    done = False

    while not done:
        # ask for input
        i_original = input('>> ')
        i = i_original.upper()

        # split input into list [command, argument]
        i = i.split(' ')
        i_original = i_original.split(' ')

        # get all possible commands
        cmds = [i[0] for i in PAYLOAD_MENU_ITEMS + BOX_MENU_ITEMS + ADMIN_MENU_ITEMS]
        # get all arguments
        arguments = [i[1] for i in PAYLOAD_MENU_ITEMS + BOX_MENU_ITEMS + ADMIN_MENU_ITEMS]

        valid_command = True

        # check if command exists
        try: 
            cmds.index(i[0])
        except Exception:
            valid_command = False

        if not valid_command:
            logger.info('Command "' + i[0] + '" not recognized.')
        # check if trying to send argument when none should be sent
        elif  len(i[1:]) != 0 and len(arguments[cmds.index(i[0])]) == 0:
            logger.info(i[0] + ' takes no arguments.')
        # check if not sending argument when one should be sent
        elif len(i[1:]) == 0 and len(arguments[cmds.index(i[0])]) != 0:
            logger.info(i[0] + ' takes arguments: ' + str(arguments[cmds.index(i[0])]))
        # check for exiting command mode and returning to listening mode
        elif i[0] == 'X':
            logger.info('Exited COMMAND MODE.')
            done = True
        # check for quitting
        elif i[0] == 'Q':
            logger.info('Program terminated.')
            sys.exit(0)
        # check for menu printing
        elif i[0] == 'M':
            print_menu()
        # check if need passthrough mode    
        elif i[0] == 'PASS':
            # check if no command was sent
            if len(i) < 2 or i[1] == '':
                logger.info('Please input command to passthrough. e.g. PASS $PWR,ON;')
            else:
                i_original[1] = ' '.join(str(e) for e in i_original[1:])
                logger.debug('Command input: ' + i_original[0] + ', ' + i_original[1] if len(i) > 1 else 'Command: ' + i_original[0])
                msg = i_original[1]
                serial_send(msg) 
                logger.warning('\t TX: '+ msg)
                done = True
        # otherwise, regular command that will be sent in '$XXX,YYY;' format
        elif True:
            logger.debug('Command input: ' + i[0] + ', ' + i[1] if len(i) > 1 else 'Command: ' + i[0])
            msg = COMMAND_BOM_CHAR + i[0] + DELIM
            if (len(i) > 1):
                msg += i[1] 
            msg += EOM_CHAR
            serial_send(msg)
            logger.warning('\t TX: '+ msg)
            done = True

# $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ SERIAL COMM HELPERS $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
# sets up serial port
def setup_serial(args):
    global ser

    successful = False # track if connection is successful
    first_failed = False # track if first try is successful or not, so can prompt
    port = args['serial_port']

    while not successful:
        if port == DEFAULT_SERIAL_PORT or first_failed:
            logger.info('The following ports are available: ')
            ports = serial.tools.list_ports.comports()
            logger.info([port.description for port in ports])
            port = input("Specify serial port (or <q> to quit) >> ").upper()
        if port == 'Q':
            logger.info('Program terminated.')
            sys.exit()
        try:        
            ser = serial.Serial(port, args['baud_rate'])
            successful = True
        except serial.serialutil.SerialException:
            logger.error('Serial port ' + port + ' in use or incorrect.')
            first_failed = True
            
    # Wait half a second to initialize
    sleep(0.5)

    # Confirm that the connection is established
    if ser.isOpen():
        logger.info('Serial port ' + port + ' opened successfully.')


# encodes a message using utf-8 and sends over serial
def serial_send(msg):
    try:
        ser.write(msg.encode('utf-8'))
    except serial.serialutil.SerialException:
        logger.error('Serial port disconnected.')

# waits for data to come in buffer, then read all data
def serial_read(disable_rec_time=False):
    global last_rec
    corrupt_count = 0
    out = ''
    try:
        while ser.in_waiting > 0:
            try:
                c = ser.read(1).decode('utf-8')
                # if reach the end of a message break so not all messages clumped up
                if c == EOM_CHAR:
                    break
                out += c
                sleep(0.01)

                # to filter out if just listening to local radio (+++, AT commands) 
                if not disable_rec_time:
                    last_rec = datetime.now()
            # catch corrupted transmissions
            except UnicodeDecodeError:
                out += '?'
                corrupt_count += 1
    except serial.serialutil.SerialException:
        logger.error('Serial port disconnected.')

    if corrupt_count != 0:
        logger.error('Message has ' + str(corrupt_count) + ' corruption(s).')
    return out

# sends a heartbeat and waits to hear back
def send_heartbeat():
    # logger.debug('Heartbeat disabled.')
    # return
    global last_rec
    # BELOW COMMENTED FOR TESTING
    serial_send(HEARTBEAT_MSG)

    logger.warning('\t TX (Heartbeat): ' + HEARTBEAT_MSG)

    # wait for heartbeat response to come in
    try:
        start = datetime.now()
        while ser.in_waiting == 0:
            if (datetime.now() - start).seconds > HEARTBEAT_TIMEOUT:
                if last_rec == None:
                    last = 'never'
                    logger.error('Did not receive heartbeat data from payload within ' + str(HEARTBEAT_TIMEOUT) + 's. Last heard: ' + last)
                else:
                    last = last_rec.strftime('%H-%M-%S.%f')[:-3]
                    diff = '{:.2f}'.format((datetime.now() - last_rec).seconds / 60.0)
                    logger.error('Did not receive data from payload within ' + str(HEARTBEAT_TIMEOUT) + 's. Last heard: ' + diff + ' minutes ago.')
                break

        if ser.in_waiting > 0:
            logger.warning('\t RX (Heartbeat): ' + serial_read())
            last_rec = datetime.now()
    except serial.serialutil.SerialException:
        logger.error('Serial port disconnected.')  

# configures xTend radio to operate at 10kb/s 
def config_radio():
    logging.debug('Attepmting to enter radio command mode.')
    serial_send('+++')
    while ser.in_waiting == 0:
        pass
    while ser.in_waiting > 0:
        logging.debug(ser.read(1).decode('utf-8'))
        sleep(0.01)
    # radio_wait()
    # ok = serial_read(disable_rec_time=True)
    # logging.error(ok)
    # if ok != 'OK':
    #     logging.error('Failed to enter radio command mode.')
    # else:
    #     logging.debug('success')

def radio_wait():
    start = datetime.now()
    while ser.in_waiting == 0: 
        if (datetime.now() - start).seconds > RADIO_COMMAND_TIMEOUT:
            logging.error('Radio did not respond to command in ' + str(RADIO_COMMAND_TIMEOUT) + ' seconds.')
            break

# $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ MENU HELPERS $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
# prints all menu items (printed only to stdout, not to log file)
def print_menu():
    # put all sub menus into one
    menus = [('Payload commands: ', PAYLOAD_MENU_ITEMS),
            ('Box commands: ', BOX_MENU_ITEMS),
            ('Admin commands: ', ADMIN_MENU_ITEMS)]

    print('--------------------------------------------------------')
    print('Separate command and argument with space. e.g. PWR ON')
    for i in range(len(menus)):
        # print submenu title
        print('\t{:20}'.format(menus[i][0]))

        # print all commands in submenus
        for j in range(len(menus[i][1])):
            print('\t    {:12} | {:18} | {}'.format(menus[i][1][j][0], ','.join(str(e) for e in menus[i][1][j][1]),menus[i][1][j][2]))

    print('--------------------------------------------------------')

def build_completer():
    # store all 'command': 'argument'
    cmds = {}

    # put all sub menus into one
    menus = [PAYLOAD_MENU_ITEMS, BOX_MENU_ITEMS, ADMIN_MENU_ITEMS]

    for i in range(len(menus)):
        # print all commands in submenus
        for j in range(len(menus[i])):
            cmds[menus[i][j][0]] = menus[i][j][1]

    # Register our completer function
    readline.set_completer(completer.Completer(cmds).complete)

    # Use the tab key for completion
    readline.parse_and_bind('tab: complete')

# $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ CL ARGS HELPERS $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
# parses command line arguments
def parse_arguments(argv):
    default_filename = datetime.now().strftime('%m-%d-%Y %I-%M-%S %p')+'.log'

    parser = argparse.ArgumentParser()

    # filename handling
    parser.add_argument('-l',
        nargs='?',
        dest='log_filename',
        const=default_filename,
        default=default_filename,
        help='set custom log filename. default: m-d-Y I-M-S p.log')

    # baud rate handling
    parser.add_argument('-R',
        nargs='?',
        dest='baud_rate',
        const=DEFAULT_BAUD_RATE,
        default=DEFAULT_BAUD_RATE,
        help='set custom serial baud rate. default: 9600')
    
    # serial port handling
    parser.add_argument('-p',
        nargs='?',
        dest='serial_port',
        const=DEFAULT_SERIAL_PORT,
        default=DEFAULT_SERIAL_PORT,
        help='set serial port name. will prompt in interface if not specified')

    # suppress logging
    parser.add_argument('-D',
        dest='debug_mode',
        action='store_true',
        help='enable debug mode, suppresses file logging')
    
    # set logging level
    parser.add_argument('-L',
        nargs='?',
        dest='logging_level',
        const=DEFAULT_LOGGING_LEVEL,
        default=DEFAULT_LOGGING_LEVEL,
        help='set logging level. default: DEBUG')
        
    return vars(parser.parse_args(argv))

# $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ LOGGER HELPERS $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
# configures logging
def config_logger(args):  
    # get logging level
    level = args['logging_level'].upper()    
    if level == 'CRITICAL':
        level = logging.CRITICAL
    elif level == 'ERROR':
        level = logging.ERROR
    elif level == 'WARNING':
        level = logging.WARNING
    elif level == 'INFO':
        level = logging.INFO
    else:
        level = logging.DEBUG
    
    # configure logging:
    global logger
    logger = logging.getLogger('ground')
    logger.setLevel(level)

    coloredlogs.install(level='DEBUG', fmt='%(asctime)s,%(msecs)03d %(levelname)s %(message)s')

    # create formatter
    formatter = logging.Formatter(fmt='%(levelname)8s \t %(asctime)s %(message)s')

    # BELOW COMMENTED OUT DUE TO INTRODUCTION OF COLOREDLOGS, WHICH OUTPUTS PRETTY
    # FORMATTED LOG RECORDS TO CONSOLE
    # # create console handler
    # ch = logging.StreamHandler()
    # ch.setLevel(logging.INFO)
    # #ch.setFormatter(formatter) # commented out to avoid cluttering of console
    # logger.addHandler(ch)

    if not args['debug_mode']:
        # create file handler
        fh = logging.FileHandler(args['log_filename'])
        fh.setLevel(level)
        fh.setFormatter(formatter)
        logger.addHandler(fh)


if __name__ == '__main__':
    main(parse_arguments(sys.argv[1:]))