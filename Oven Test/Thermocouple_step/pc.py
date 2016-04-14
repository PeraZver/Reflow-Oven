# Pero 2015
#
# Interface with the ATmega32u2 based thermometer.
# The program gets the temperature step response of the oven.
# and can write duty cycle value to ATmega's RAM and EEPROM.
# Based on the code from Henrik Forsten
# 
# April 2016
#


import serial
import sys
import argparse
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

BAUDRATE = 115200
PARITY = False
PWM_MAX = 6250

class AnalogPlot:
    def __init__(self, ser, maxlen):
        self.x = deque([0]*maxlen, maxlen)
        self.y1 = deque([0]*maxlen, maxlen)
        self.y2 = deque([0]*maxlen, maxlen)
        self.maxlen = maxlen
        self.starttime = time.time()
        self.ser = ser

    def add(self, status):
        #Only add datapoint when it's possible to add in
        #both plots
        if 'temp' in status:
            self.x.appendleft(time.time() - self.starttime)
            self.y1.appendleft(status['temp'])
            self.y2.appendleft(status['pwm']*660 + status['room'])

    def update(self, frameNum, p0, p1, ax):
        try:
            status = get_status(self.ser)
            self.add(status)
            print_status(status)
            fajl.write(str(self.x[1]))
            fajl.write('\t')
            fajl.write(str(status['temp']))
            fajl.write('\n')
            #fajl.close()
            
        except TypeError:
            pass
        ax.set_xlim(self.x[-1], max(10,self.x[0]))
        ax.set_ylim((0, max(250,max(self.y2),max(self.y1))))
        p0.set_data(self.x, self.y1)
        p1.set_data(self.x, self.y2)
        return (p0,p1,ax)

#Configures serial port
def configure_serial(serial_port):
    return serial.Serial(
        port=serial_port,
        baudrate=BAUDRATE,
        parity=serial.PARITY_EVEN if PARITY else serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        timeout=0.05)

def u16_to_chars(u16):
    """uint16_t to two bytes"""
    if u16>2**16 or u16<0:
        raise ValueError
    a,b = u16>>8,u16&(~(0xff<<8))
    return chr(a),chr(b)

def write_profile(ser, settings):
    """Writes a correctly formatted list of settings to device"""
    u16s = map(u16_to_chars,settings)
    #Flatten the list
    u16s = [item for sublist in u16s for item in sublist]
    u16s = ''.join(u16s)
    ser.write('!W'+u16s)

def set_pid(ser, settings):
    if len(settings) != 1:
        raise ValueError("set_pid needs 1 values")
    settings = map(int,settings)   # converts to integer
    print settings
    write_profile(ser, settings)
    print "Writing DUTY CYCLE...."
    time.sleep(5)
    print "New PWM:"
    print ser.readline()
  #  ser.close()
 #   print_profile(get_profile(ser)

def get_status(ser):
    s = ser.readline()
    s = s.strip()
    s = s.replace(':',',')
    status = s.split(',')
    if len(status) == 6:
        try:
            temp = float(status[1])
            room = float(status[3])
            pwm = float(status[5])/PWM_MAX
            return {'temp':temp, 'room':room, 'pwm':pwm}
        except:
            print 'Neki kurac nevalja u get_status'
            pass
    return None

def print_status(status):
    try:
        print 'Temp: {temp:3.2f}, Room: {room:3.2f}, PWM: {pwm:1.2f}'.format(**status)
    except (TypeError, KeyError):
        print 'Neki kurac nevalja u print_status'

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PID controller for Reflow oven")
    parser.add_argument('-p','--set_pid', nargs=3, metavar=('P'), type=float, help='Set duty cycle: 0 - 6250')
    parser.add_argument('-g','--get', action='store_true', help='Get the current profile')
    parser.add_argument('-l','--plot', dest='plot', action='store_true', help="Draw plot")
    parser.add_argument('-n','--no-plot', dest='plot', action='store_false', help="Don't draw plot")
    parser.add_argument('port',metavar='Port', help='Serial port')
    parser.set_defaults(get=False, plot=True)
    args = parser.parse_args()

    if len(sys.argv)<2:
        print "Give serial port address as a command line argument."
        exit()
    try:
        ser = configure_serial(args.port)
        if not ser.isOpen():
            raise Exception
    except:
        print 'Opening serial port {} failed.'.format(args.port)
        exit(1)
        
    print('reading from serial port %s...' %args.port)

    if args.plot:
        fajl = open('TempTest.txt','w')
        analogPlot = AnalogPlot(ser, 3000)

        fig = plt.figure()
        ax = plt.axes(xlim=(0, 100), ylim=(0, 300))
        p0, = ax.plot([], [])
        p1, = ax.plot([], [])        
        
        anim = animation.FuncAnimation(fig, analogPlot.update,
                                         fargs=(p0,p1,ax),
                                         interval=10)
        plt.show()
        exit()

    while True:
        try:
            try:
                print_status(get_status(ser))
            except serial.serialutil.SerialException:
                continue
            except OSError:
                continue
            except KeyError:
                continue
        except KeyboardInterrupt:
            ser.close()
            fajl.close()
            break
