#!/usr/bin/python
# snakeoil.py
# Chris X Edwards <snakeoil@xed.ch>
# Snake Oil is a Python library for interfacing with a TORCS
# race car simulator which has been patched with the server
# extentions used in the Simulated Car Racing competitions.
# http://scr.geccocompetitions.com/
#
# To use it, you must import it and create a "drive()" function.
# This will take care of option handling and server connecting, etc.
# To see how to write your own client do something like this which is
# a complete working client:
# /-----------------------------------------------\
# |#!/usr/bin/python                              |
# |import snakeoil                                |
# |if __name__ == "__main__":                     |
# |    C= snakeoil.Client()                       |
# |    for step in xrange(C.maxSteps,0,-1):       |
# |        C.get_servers_input()                  |
# |        snakeoil.drive_example(C)              |
# |        C.respond_to_server()                  |
# |    C.shutdown()                               |
# \-----------------------------------------------/
# This should then be a full featured client. The next step is to
# replace 'snakeoil.drive_example()' with your own. There is a
# dictionary which holds various option values (see `default_options`
# variable for all the details) but you probably only need a few
# things from it. Mainly the `trackname` and `stage` are important
# when developing a strategic bot. 
#
# This dictionary also contains a ServerState object
# (key=S) and a DriverAction object (key=R for response). This allows
# you to get at all the information sent by the server and to easily
# formulate your reply. These objects contain a member dictionary "d"
# (for data dictionary) which contain key value pairs based on the
# server's syntax. Therefore, you can read the following:
#    angle, curLapTime, damage, distFromStart, distRaced, focus,
#    fuel, gear, lastLapTime, opponents, racePos, rpm,
#    speedX, speedY, speedZ, track, trackPos, wheelSpinVel, z
# The syntax specifically would be something like:
#    X= o[S.d['tracPos']]
# And you can set the following:
#    accel, brake, clutch, gear, steer, focus, meta 
# The syntax is:  
#     o[R.d['steer']]= X
# Note that it is 'steer' and not 'steering' as described in the manual!
# All values should be sensible for their type, including lists being lists.
# See the SCR manual or http://xed.ch/help/torcs.html for details.
#
# If you just run the snakeoil.py base library itself it will implement a
# serviceable client with a demonstration drive function that is
# sufficient for getting around most tracks.
# Try `snakeoil.py --help` to get started.

import socket 
import sys
import getopt
import numpy as np
from collections import namedtuple
from tkinter import *

PI= 3.14159265359

# Initialize help messages
ophelp=  'Options:\n'
ophelp+= ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp+= ' --port, -p <port>    TORCS port. [3001]\n'
ophelp+= ' --id, -i <id>        ID for server. [SCR]\n'
ophelp+= ' --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n'
ophelp+= ' --episodes, -e <#>   Maximum learning episodes. [1]\n'
ophelp+= ' --track, -t <track>  Your name for this track. Used for learning. [unknown]\n'
ophelp+= ' --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp+= ' --debug, -d          Output full telemetry.\n'
ophelp+= ' --help, -h           Show this help.\n'
ophelp+= ' --version, -v        Show current version.'
usage= 'Usage: %s [ophelp [optargs]] \n' % sys.argv[0]
usage= usage + ophelp
version= "20130505-2"

class Client():
    def __init__(self,H=None,p=None,i=None,e=None,t=None,s=None,d=None):
        # If you don't like the option defaults,  change them here.
        self.host= 'localhost'
        self.port= 3001
        self.sid= 'SCR'
        self.maxEpisodes=1
        self.trackname= 'unknown'
        self.stage= 3
        self.debug= False
        self.maxSteps= 100000  # 50steps/second
        self.parse_the_command_line()
        if H: self.host= H
        if p: self.port= p
        if i: self.sid= i
        if e: self.maxEpisodes= e
        if t: self.trackname= t
        if s: self.stage= s
        if d: self.debug= d
        self.S= ServerState()
        self.R= DriverAction()
        self.setup_connection()

    def setup_connection(self):
        global angles
        # == Set Up UDP Socket ==
        try:
            self.so= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket...')
            sys.exit(-1)
        # == Initialize Connection To Server ==
        self.so.settimeout(1)
        while True:

            #a= "-90 -75 -60 -45 -30 -20 -15 -10 -5 0 5 10 15 20 30 45 60 75 90"
            a = ' '.join(str(angle) for angle in angles)
            initmsg='%s(init %s)' % (self.sid,a)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                sys.exit(-1)
            sockdata= str()
            try:
                sockdata,addr= self.so.recvfrom(1024)
                sockdata = sockdata.decode()
            except socket.error as emsg:
                #print("Waiting for server............")
                continue
            if '***identified***' in sockdata:
                #print("Client connected..............")
                break

    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(sys.argv[1:], 'H:p:i:m:e:t:s:dhv',
                       ['host=','port=','id=','steps=',
                        'episodes=','track=','stage=',
                        'debug','help','version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] == '-h' or opt[0] == '--help':
                    print(usage)
                    sys.exit(0)
                if opt[0] == '-d' or opt[0] == '--debug':
                    self.debug= True
                if opt[0] == '-H' or opt[0] == '--host':
                    self.host= opt[1]
                if opt[0] == '-i' or opt[0] == '--id':
                    self.sid= opt[1]
                if opt[0] == '-t' or opt[0] == '--track':
                    self.trackname= opt[1]
                if opt[0] == '-s' or opt[0] == '--stage':
                    self.stage= opt[1]
                if opt[0] == '-p' or opt[0] == '--port':
                    self.port= int(opt[1])
                if opt[0] == '-e' or opt[0] == '--episodes':
                    self.maxEpisodes= int(opt[1])
                if opt[0] == '-m' or opt[0] == '--steps':
                    self.maxSteps= int(opt[1])
                if opt[0] == '-v' or opt[0] == '--version':
                    print('%s %s' % (sys.argv[0], version))
                    sys.exit(0)
        except ValueError as why:
            print('Bad parameter {0} for option {1}: {2}\n{3}'.format(opt[1], opt[0], why, usage))
            sys.exit(-1)
        if len(args) > 0:
            print('Superflous input? {0}\n{1}'.format(', '.join(args), usage))
            sys.exit(-1)

    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata= str()
        while True:
            try:
                # Receive server data 
                sockdata,addr= self.so.recvfrom(1024)
                sockdata = sockdata.decode()
            except socket.error as emsg:
                #print("Waiting for data..............")
                continue
            if '***identified***' in sockdata:
                #print("Client connected..............")
                continue
            elif '***shutdown***' in sockdata:
                #print("Server has stopped the race. You were in {0} place.".format(self.S.d['racePos']))
                self.shutdown()
                return -2
            elif '***restart***' in sockdata:
                # What do I do here?
                #print("Server has restarted the race.")
                # I haven't actually caught the server doing this.
                self.shutdown()
                return -1
            elif not sockdata: # Empty?
                continue       # Try again.
            else:
                self.S.parse_server_str(sockdata)
                if self.debug: print(self.S)
                break # Can now return from this function.
        return 0

    def respond_to_server(self):
        if not self.so: return
        if self.debug: print(self.R)
        try:
            self.so.sendto(repr(self.R).encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: {0} Message {1}".format(emsg[1],str(emsg[0])))
            sys.exit(-1)

    def shutdown(self):
        if not self.so: return
        #print("Race terminated or {0} steps elapsed. Shutting down.".format(self.maxSteps))
        self.so.close()
        self.so= None
        #sys.exit() # No need for this really.

class ServerState():
    'What the server is reporting right now.'
    def __init__(self):
        self.servstr= str()
        self.d= dict()

    def parse_server_str(self, server_string):
        'parse the server string'
        self.servstr= server_string.strip()[:-1]
        sslisted= self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w= i.split(' ')
            self.d[w[0]]= destringify(w[1:])

    def __repr__(self):
        out= str()
        for k in sorted(self.d):
            strout= str(self.d[k])
            if type(self.d[k]) is list:
                strlist= [str(i) for i in self.d[k]]
                strout= ', '.join(strlist)
            out+= "%s: %s\n" % (k,strout)
        return out

class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)'''
    def __init__(self):
       self.actionstr= str()
       # "d" is for data dictionary.
       self.d= { 'accel':0.2,
                   'brake':0,
                  'clutch':0,
                    'gear':1,
                   'steer':0,
                   'focus':[-90,-45,0,45,90],
                    'meta':0 
                    }

    def __repr__(self):
        out= str()
        for k in self.d:
            out+= '('+k+' '
            v= self.d[k]
            if not type(v) == list:
                out+= '%.3f' % v
            else:
                out+= ' '.join([str(x) for x in v])
            out+= ')'
        return out
        return out+'\n'

# == Misc Utility Functions
def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in {0}".format(s))
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]

def clip(v,lo,hi):
    if v<lo: return lo
    elif v>hi: return hi
    else: return v

def drive_example(c):
    '''This is only an example. It will get around the track but the
    correct thing to do is write your own `drive()` function.'''
    S= c.S.d
    R= c.R.d
    target_speed=100

    # Damage Control
    target_speed-= S['damage'] * .05
    if target_speed < 25: target_speed= 25

    # Steer To Corner
    R['steer']= S['angle']*10 / PI
    # Steer To Center
    R['steer']-= S['trackPos']*.10
    R['steer']= clip(R['steer'],-1,1)

    # Throttle Control
    if S['speedX'] < target_speed - (R['steer']*50):
        R['accel']+= .01
    else:
        R['accel']-= .01
    if S['speedX']<10:
       R['accel']+= 1/(S['speedX']+.1)

    # Traction Control System
    if ((S['wheelSpinVel'][2]+S['wheelSpinVel'][3]) -
       (S['wheelSpinVel'][0]+S['wheelSpinVel'][1]) > 5):
       R['accel']-= .2
    R['accel']= clip(R['accel'],0,1)

    # Automatic Transmission
    R['gear']=1
    if S['speedX']>50:
        R['gear']=2
    if S['speedX']>80:
        R['gear']=3
    if S['speedX']>110:
        R['gear']=4
    if S['speedX']>140:
        R['gear']=5
    if S['speedX']>170:
        R['gear']=6
    return

def calcWheelSlip (carSpeedKph, wheelSpinVel):
    frontSlip = 0.0
    rearSlip = 0.0
    carSpeed = carSpeedKph / 3.6 # convert to m/s
    if carSpeed > 0.001:
        slips = []
        for i in range(4):
            wheelSurfaceSpeed = wheelSpinVel[i] * 0.3306
            difference = wheelSurfaceSpeed - carSpeed
            proportionalDifference = difference / carSpeed
            slips.append(proportionalDifference)
        frontSlip = np.mean(slips[0:2]) # only front wheels for ABS
        rearSlip = np.mean(slips[2:4]) # only rear wheels for TCL
        avgSlip = np.mean(slips)
        #print("wheel speeds relative to ground: {} (avg: {:.2f})".format(' '.join(["{0:.2f}".format(i) for i in slips]), avgSlip))
    return frontSlip, rearSlip

def brakeAbs (brakePressure, frontSlip):
    absTarget = -0.25
    absControl = 0.25
    brakePressure += (frontSlip - absTarget) * absControl
    brakePressure = np.clip(brakePressure, 0.0, 1.0)
    return brakePressure

def accelTcl (accelPressure, rearSlip):
    tclTarget = 0.3
    tclControl = 0.25
    accelPressure -= (rearSlip - tclTarget) * tclControl
    accelPressure = np.clip(accelPressure, 0, 1)
    return accelPressure

def autoTransmission (gear, rpm, rearSlip):
    upshiftRpm = 9600
    downshiftRpm = 3500
    maximumSlip = 0.1
    if gear < 6 and rpm > upshiftRpm and rearSlip < maximumSlip:
        return gear + 1
    elif gear > 1 and rpm < downshiftRpm:
        return gear - 1
    return gear

def steerToCenter (trackPos):
    steering = trackPos * -0.50
    steering = clip(steering, -1, 1)
    return steering

def steerToCorner (angle):
    steering = angle*5 / PI
    return steering

def drive100msprint (c):
    S= c.S.d
    R= c.R.d
    global hasReportedTime
    global hasReportedDistance
    global brakePressure, accelPressure


    carSpeed = S['speedX'] / 3.6
    frontSlip, rearSlip = calcWheelSlip(S['speedX'], S['wheelSpinVel'])

    R['steer'] = steerToCorner(S['angle'])
    R['gear'] = autoTransmission (S['gear'], S['rpm'], rearSlip)


    distance = 300
    hasFinished = S['distRaced'] > distance
    if hasFinished:
        R['accel'] = 0.0

        brakePressure = brakeAbs(brakePressure, frontSlip)
        R['brake'] = brakePressure

        if not hasReportedTime:
            timeTo100m = S['curLapTime']
            print("{}m reached in {:.2f}s".format(distance, timeTo100m))
            hasReportedTime = True

        hasStopped = S['speedX'] < 0.1
        if hasStopped and not hasReportedDistance:
            hasReportedDistance = True
            brakeDistance = S['distRaced'] - distance
            print("braking distance: {:.2f}m".format(brakeDistance))
            R['meta'] = 1 # restart race
            return brakeDistance
    elif carSpeed > 3: # slip is noisy at low speeds
        accelPressure = accelTcl(accelPressure, rearSlip)
        R['accel'] = accelPressure
    else:
        R['accel'] = 1.0

    return -1


def race (c):
    S= c.S.d
    R= c.R.d

    furthestScanDistance = np.max(S['track'])
    furthestScanAngle = angles[np.argmax(S['track'])]

    #shouldAccellerate = furthestScanDistance > 100
    #shouldSlowDown = furthestScanDistance < 100 and 
    R['accel'] = 1.0
    R['brake'] = 0.0
    if S['track'][9] < 150.0:
        R['accel'] -= (150-S['track'][9]) / 150
        if S['speedX'] > 100 and S['track'][9] < 50:
            R['brake'] = max((S['track'][9] - 20) / 50, 0)
            if R['brake'] > 0.01:
                R['accel'] = 0.0

    if S['speedY'] > 5: # screeeech
        R['accel'] = 0.0

    frontSlip, rearSlip = calcWheelSlip(S['speedX'], S['wheelSpinVel'])
    R['steer'] = steerToCorner(S['angle']) + steerToCenter(S['trackPos'])
    R['gear'] = autoTransmission (S['gear'], S['rpm'], rearSlip)

    # updateTrackDisplay(S['track'])
    # updateEnemyDisplay(S['opponents'])

def cruise (c):
    S= c.S.d
    R= c.R.d
    global accelPressure

    targetSpeed = 100 # kph
    targetSpeed -= 30 * abs(S['angle'])
    targetSpeed = max(targetSpeed, 30)
    targetSpeed /= 3.6 # m/s

    carSpeed = S['speedX'] / 3.6
    accelPressure += (targetSpeed - carSpeed) * 0.1
    accelPressure = clip(accelPressure, 0, 1)
    R['accel'] = accelPressure
    R['brake'] = 0.0
    R['steer'] = steerToCorner(S['angle']) + steerToCenter(S['trackPos'])
    R['gear'] = autoTransmission (S['gear'], S['rpm'], 0.0)

    # updateTrackDisplay(S['track'])
    # updateEnemyDisplay(S['opponents'])

    return

def updateTrackDisplay (trackdata):
    global zoom, carCenter, screenSize
    sidelinePoints = []
    sidelinePoints.append([]) # left
    sidelinePoints.append([]) # right
    roadPoints = []

    maxIndex = np.argmax(trackdata)
    for i in range(19):
        # update range lines
        theta = angles[i] * PI/180.0
        R = [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]];
        
        distance = trackdata[i] * zoom
        secondPoint = np.array([0,-distance])
        secondPoint = (R @ secondPoint) + carCenter
        w.coords(SCR['rangeLines'][i], *carCenter, *tuple(secondPoint))

        # save for road surface polygon
        roadPoints.append(secondPoint)

        # assign it to left or right side of track
        if i <= maxIndex and trackdata[i] < 200:
            sidelinePoints[0].append(secondPoint)
        elif i > maxIndex and trackdata[i] < 200:
            sidelinePoints[1].append(secondPoint)

        # draw furthest scan as indicator for racing driver
        isFurthestScan = i == maxIndex
        if isFurthestScan:
            w.itemconfig(SCR['rangeLines'][i], fill='blue')
        else:
            w.itemconfig(SCR['rangeLines'][i], fill='#888')

    # update road polygon
    magic = [e for l in roadPoints for e in l]
    w.coords(SCR['road'], magic)

    # update road side lines
    magic = [e for l in sidelinePoints[0] for e in l]
    if len(magic) > 3:
        w.coords(SCR['sidelines'][0], magic)
    magic = [e for l in sidelinePoints[1] for e in l]
    if len(magic) > 3:
        w.coords(SCR['sidelines'][1], magic)

    # update background to force redraw
    w.coords(SCR['background'], [0, 0, screenSize.width+1, screenSize.height+1])


def updateEnemyDisplay (opponents):
    Point = namedtuple('Point', 'x y')
    global carCenter
    for i in range(36):
        # theta = -PI + i * PI/18 + PI/36

        # # update range lines for enemies
        # R = [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]];
        
        distance = opponents[i]
        if distance > 199.9: # no enemy in range
            distance = 0.0
        distance *= zoom
        # secondPoint = np.array([0,-distance])
        # secondPoint = (R @ secondPoint) + carCenter
        # w.coords(SCR['enemyLines'][i], *carCenter, *tuple(secondPoint))
        topLeftPoint = Point(carCenter.x-distance, carCenter.y-distance)
        topRightPoint = Point(carCenter.x+distance, carCenter.y+distance)
        w.coords(SCR['enemyArcs'][i], *topLeftPoint, *topRightPoint)


def reset ():
    # 100m sprint related
    global hasReportedTime, hasReportedDistance
    hasReportedTime = False
    hasReportedDistance = False

def init ():
    reset()

    global brakePressure, accelPressure
    brakePressure = 0.2
    accelPressure = 1.0
    
    # debug drawing screen
    # global zoom, carCenter, screenSize
    # Point = namedtuple('Point', 'x y')
    # Box = namedtuple('Box', 'width height')
    # zoom = 4.0
    # screenSize = Box(600, 1000)
    # carSize = Box(1.94, 4.52)
    # carCenter = Point(0.5*screenSize.width, 0.9*screenSize.height)

    calculateLrfAngles()

    # global master,w,SCR
    # master = Tk()
    # w = Canvas(master, width=screenSize.width, height=screenSize.height)
    # w.pack()
    # SCR = {}
    # SCR['background'] = w.create_rectangle(0, 0, screenSize.width+1, screenSize.height+1, fill='#018E0E')
    # SCR['road'] = w.create_polygon([0,0,0,0], fill='#777', smooth='1')
    # SCR['sidelines'] = []
    # for i in range(2):
    #     SCR['sidelines'].append(w.create_line(carCenter, carCenter, fill='#CCC', width='3', smooth='1'))
    # SCR['rangeLines'] = []
    # for i in range(19):
    #     SCR['rangeLines'].append(w.create_line(carCenter, carCenter, fill='#888'))
    # SCR['enemyArcs'] = []
    # for i in range(36):
    #     thetaStart = -PI/2 - (i+1) * PI/18
    #     thetaStart *= 180.0/PI
    #     extent = 10
    #     SCR['enemyArcs'].append(w.create_arc(carCenter, carCenter, start=thetaStart, extent=extent, style='arc', width='2', outline='red'))
    # SCR['car'] = w.create_rectangle(
    #     carCenter.x - 0.5*carSize.width*zoom, 
    #     carCenter.y - 0.5*carSize.height*zoom, 
    #     carCenter.x + 0.5*carSize.width*zoom, 
    #     carCenter.y + 0.5*carSize.height*zoom, fill="blue")

    # map drawing screen
    #mapWindowSize = Box(1000, 1000)
    #canvas = Canvas(master, width=mapWindowSize.width, height=mapWindowSize.height, bg="#000")
    #canvas.pack()
    #img = PhotoImage(width=mapWindowSize.width, height=mapWindowSize.height)
    #canvas.create_image((mapWindowSize.width/2, mapWindowSize.height/2), image=img, state="normal")

def calculateLrfAngles ():
    global angles
    angles = []
    roadWidth = 11
    sensorFocus = 25
    sampleSpread = (np.logspace(0, 1.0, base=sensorFocus, num=9) - 1)/(sensorFocus-1)
    #print(sampleSpread)
    for i in range(19):
        if i < 9:
            angle = np.arctan((sampleSpread[i]*200.0) / (roadWidth/2))
            angle = ((-PI/2.0) + angle) * 180/PI
        elif i == 9:
            angle = 0
        elif i > 9:
            angle = np.arctan((sampleSpread[(9-(i-9))]*200.0) / (roadWidth/2))
            angle = ((PI/2.0) - angle) * 180/PI
        #print("angle {0}: {1}".format(i, angle))
        angles.append(angle)
    #angles = [-90, -65, -50, -35, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 35, 50, 65, 90]
    #print(' '.join(str(angle) for angle in angles))


def keepRepeatingRace (driver):
    while True:
        reset()
        flag = doRace(driver)
        if flag == -2: # exit
            break

def doRace (driver):
    C= Client()
    for step in range(C.maxSteps,0,-1):
        flag = C.get_servers_input()
        if flag < 0:
            break
        driver(C)
        C.respond_to_server()
        #if 'normal' == master.state():
            # w.update()
            #canvas.update()
    C.shutdown()
    return flag

# ================ MAIN ================
if __name__ == "__main__":
    init()
    #keepRepeatingRace(drive100msprint)
    #doRace(cruise)
    doRace(race)
    w.destroy()
    master.quit()   
