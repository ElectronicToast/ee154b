from time import sleep, time
import serial, random, string, sys, logging, argparse, readline, coloredlogs
from datetime import datetime
import completer as completer
import kbhit as kbhit
import serial.tools.list_ports

# Constants
DEFAULT_BAUD_RATE = 9600
DEFAULT_SERIAL_PORT = 'COM3' # on windows
DEFAULT_LOGGING_LEVEL = 'DEBUG'
COMMAND_BOM_CHAR = '$' # beginning of message character for commands
TELEM_BOM_CHAR = '#' # beginning of message character for received telemetry
EOM_CHAR = ';' # marks end of time temperature data
DELIM = ',' # delimiter between commands and arguments
HEART_BEAT_INTERVAl = 5 # time in seconds between heartbeats
HEART_BEAT_TIMEOUT = 1 # time to wait for heartbeat response

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
    ('MOTR', ['0-100'], 'Set instrument flywheel speed.')]

BOX_MENU_ITEMS = [
    ('DOOR', [], 'Force open the side panel.'),
    ('PASS', [], 'Pass through any data.'),
    ('KP', ['int'], 'Change the Proportional constant for temperature PID loop.'),
    ('KI', ['int'], 'Change the Integral constant for temperature PID loop.'),
    ('KD', ['int'], 'Change the Derivative constant for temperature PID loop.')]
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
            # check if received anything when in command mode
            if ser.in_waiting > 0:
                logger.warning('\t RX (delayed): ' + serial_read())

        # check if receiving data
        if ser.in_waiting > 0:
            logger.warning('\t RX: ' + serial_read())

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
        if (datetime.now() - last_sent_heartbeat).seconds > HEART_BEAT_INTERVAl:
            last_sent_heartbeat = datetime.now()
            send_heartbeat()

# state that prompts for command input
def command_state():
    logger.info('Entered COMMAND MODE.')
    done = False

    while not done:
        # ask for input
        i = input('>> ').upper()

        # split input into list [command, argument]
        i = i.split(' ')

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
                i[1] = ' '.join(str(e) for e in i[1:])
                logger.debug('Command input: ' + i[0] + ', ' + i[1] if len(i) > 1 else 'Command: ' + i[0])
                msg = i[1]
                serial_send(msg) 
                logger.warning('\t TX: '+ msg)
                done = True
        # otherwise, regular command that will be sent in '$XXX,YYY;' format
        elif True:
            logger.debug('Command input: ' + i[0] + ', ' + i[1] if len(i) > 1 else 'Command: ' + i[0])
            msg = COMMAND_BOM_CHAR + i[0]
            if (len(i) > 1):
                msg += DELIM + i[1]
            msg += EOM_CHAR
            serial_send(msg)
            logger.warning('\t TX: '+ msg)
            done = True

# $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ SERIAL COMM HELPERS $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
# sets up serial port
def setup_serial(args):
    # Try to open the serial port
    global ser
    try:
        logger.info('The following ports are available: ')
        ports = serial.tools.list_ports.comports()
        logger.info([port.description for port in ports])
        ser = serial.Serial(args['serial_port'], args['baud_rate'])
    except serial.serialutil.SerialException:
        logger.error('Serial port ' + args['serial_port'] + ' in use or incorrect.')
        logger.info('Program terminated.')
        sys.exit()

    # Wait half a second to initialize
    sleep(0.5)

    # Confirm that the connection is established
    if ser.isOpen():
        logger.info('Serial port ' + args['serial_port'] + ' opened successfully.')


# encodes a message using utf-8 and sends over serial
def serial_send(msg):
    ser.write(bytes(msg, 'utf-8'))

# waits for data to come in buffer, then read all data
def serial_read():
    global last_rec
    out = ''
    while ser.in_waiting > 0:
        out += ser.read(1).decode('utf-8')
        sleep(0.01)
    last_rec = datetime.now()
    return out

def send_heartbeat():
    global last_rec
    msg = '$STAT;'
    serial_send(msg)
    logger.warning('\t TX (Heartbeat): ' + msg)

    # wait for heartbeat response to come in
    start = datetime.now()
    while ser.in_waiting == 0:
        if (datetime.now() - start).seconds > HEART_BEAT_TIMEOUT:
            if last_rec == None:
                last = 'never'
                logger.error('Did not receive data from payload within ' + str(HEART_BEAT_TIMEOUT) + 's. Last heard: ' + last)
            else:
                last = last_rec.strftime('%H-%M-%S.%f')[:-3]
                diff = '{:.2f}'.format((datetime.now() - last_rec).seconds / 60.0)
                logger.error('Did not receive data from payload within ' + str(HEART_BEAT_TIMEOUT) + 's. Last heard: ' + last + ', ' + diff + ' minutes ago.')
            break
        pass

    if ser.in_waiting > 0:
        logger.warning('\t RX (Heartbeat): ' + serial_read())
        last_rec = datetime.now()

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
            print('\t    {:4} | {:15} | {}'.format(menus[i][1][j][0], ','.join(str(e) for e in menus[i][1][j][1]),menus[i][1][j][2]))

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
        help='set serial port name. default: COM3')

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