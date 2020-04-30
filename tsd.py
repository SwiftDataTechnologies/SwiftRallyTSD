#!/usr/bin/env python

# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on available packages.
async_mode = 'gevent'
bRPi = False

if async_mode is None:
    try:
        import eventlet
        async_mode = 'eventlet'
    except ImportError:
        pass

    if async_mode is None:
        try:
            from gevent import monkey
            async_mode = 'gevent'
        except ImportError:
            pass

    if async_mode is None:
        async_mode = 'threading'

print('async_mode is ' + async_mode)

# monkey patching is necessary because this application uses a background
# thread
if async_mode == 'eventlet':
    import eventlet
    eventlet.monkey_patch()
elif async_mode == 'gevent':
    from gevent import monkey
    monkey.patch_all()

if bRPi:
    import RPi.GPIO as GPIO

import sys
import os
import time
import datetime
import copy
import pandas
import numpy as np
from threading import Thread
from flask import Flask, render_template, session, request
from flask_socketio import SocketIO, emit, join_room, leave_room, \
    close_room, rooms, disconnect

tsdapp = Flask(__name__)
tsdapp.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(tsdapp, async_mode=async_mode)
thread = None

class SensorThreadClass(Thread):
    def __init__(self, sensorid):
        Thread.__init__(self)
        self.sensorid = sensorid
        self.count = 0

    def run(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        print("*********** Sensor " + str(self.sensorid) + " is alive.")
        GPIO.setup(self.sensorid, GPIO.IN, GPIO.PUD_UP)
        while True:
            time.sleep(0.001)
            if GPIO.input(self.sensorid) == False:
                self.count = self.count + 1
            elif self.count != 0:
                self.count = 0


def background_thread():
    """Main loop."""
    sensor_right = SensorThreadClass(14)
    sensor_right.start()
    sensor_left = SensorThreadClass(15)
    sensor_left.start()
    state = init_state_and_global()
    while True:
        time.sleep(0.3)
        print("***********" + str(sensor_right.sensorid))
        print("***********" + str(sensor_right.count))
        state = integrate(state, read_sensors())
        socketio.emit('tsd data', display_data(state), namespace='/test')


def integrate(state, sensors):
    """State management and integration."""
    global globalData
    # Integrate or not yet
    if not globalData['bStart']:
        return state
    elif state['t0'] == 0.0:
        state['t0'] = time.time() # Instead of doing it at the click of a button this could be prescribed by organisation
        globalData['bZeroODO'] = True
    # Waypoints
    if globalData['bAtWypt']:
        # Navigator says we just got at a waypoint
        # TODO: RECORD DATA HERE - USEFUL FOR CALIBRATION AND POST RACE ANALYSIS.
        state['xNav']    = globalData['wypts'][globalData['numWypt']][0]
        globalData['bAtWypt'] = False
    elif len(globalData['wypts']) > globalData['numWypt']+1 and state['xNav'] > globalData['wypts'][globalData['numWypt']+1][0]:
        # Update current waypoint automatically if necessary and navigator says nothing
        globalData['numWypt'] += 1
    # Current average speed target
    state['vAveTgt'] = globalData['wypts'][globalData['numWypt']][1]
    # Time
    state['tPrevious'] = state['t']
    state['t'] = sensors['t']
    dt = state['t'] - state['tPrevious']
    state['tdelta'] = (state['xTgt'] - state['xNav']) / state['vAveTgt']
    # Distance
    if globalData['bZeroODO']:
        state['x'] = 0.0
        state['xTgt'] = 0.0
        globalData['bZeroODO'] = False
    else:
        state['x']    = state['x']    + dt * sensors['v']
        state['xNav'] = state['xNav'] + dt * sensors['v']
        state['xTgt'] = state['xTgt'] + dt * state['vAveTgt']
    # Speed
    state['v'] = sensors['v']
    state['vAve'] = state['xNav'] / (0.1 + sensors['t'] - state['t0'])
    # Other
    #state['date'] = sensors['date']
    return state


def display_data(state):
    """Create strings to be displayed."""
    global globalData
    displayData = state.copy()
    displayData['t'] = datetime.datetime.fromtimestamp(state['t']).strftime('%X')
    # Unit conversions
    for field in ['x', 'xTgt', 'xNav']:
        displayData[field] = 0.001 * displayData[field]
    for field in ['v', 'vAve', 'vAveTgt']:
        displayData[field] = 3.6 * displayData[field]
    # Display precision
    for field in ['x', 'xTgt', 'xNav']:
        displayData[field] = "{:.2f}".format(displayData[field])
    for field in ['vAve', 'vAveTgt', 'tdelta']:
        displayData[field] = "{:.1f}".format(displayData[field])
    for field in ['v']:
        displayData[field] = "{:.0f}".format(displayData[field])
    # Other niceties
    for field in ['tdelta']:
        displayData[field] = displayData[field] + '"'
    # Waypoint data
    displayData['numWypt'] = globalData['numWypt']
    displayData['wypts']   = copy.deepcopy(globalData['wypts'])
    displayData['wyptTgtTime'] = [""] * 100
    t = 0.0
    displayData['wyptTgtTime'][0] = datetime.datetime.fromtimestamp(state['t0']+t).strftime('%X')
    for i in range(1,len(globalData['wypts'])):
        t += (globalData['wypts'][i][0] - globalData['wypts'][i-1][0]) / globalData['wypts'][i-1][1]
        displayData['wyptTgtTime'][i] = datetime.datetime.fromtimestamp(state['t0']+t).strftime('%X')
    for i in range(0,len(displayData['wypts'])):
        displayData['wypts'][i][0] = displayData['wypts'][i][0] / 1000.0
    return displayData 


def read_sensors():
    t = time.time()
    date = datetime.datetime.fromtimestamp(t)
    sensors = {
        't': t,
        'date': date,
        'v': 45 / 3.6
    }
    return sensors


def init_state_and_global():
    global globalData
    globalData = {
        'bZeroODO': False,
        'bStart':   False,
        'bAtWypt':  False,
        'numWypt':  0,
        'wypts':    [[0.0, 50.0 / 3.6]]
    }
    state = {
        't0':       0.0,
        't':        0.0,
        'tPrevious':0.0,
        'tdelta':   0.0,
        'x':        0.0,
        'xTgt':     0.0,
        'xNav':     0.0,
        'v':        0.0,
        'vAve':     0.0,
        'vAveTgt': 50.0 / 3.6,
        'wypt':     0,
        'date':     False
    }
    return state


@tsdapp.route('/')
def index():
    global thread
    if thread is None:
        thread = Thread(target=background_thread)
        thread.daemon = True
        thread.start()
    initialData = {}
    return render_template('codriver.html', **initialData)


@tsdapp.route('/driver')
def driver():
    global thread
    if thread is None:
        thread = Thread(target=background_thread)
        thread.daemon = True
        thread.start()
    initialData = {}
    return render_template('driver.html', **initialData)

@tsdapp.route('/config')
def config():
    global thread
    if thread is None:
        thread = Thread(target=background_thread)
        thread.daemon = True
        thread.start()
    initialData = {}
    return render_template('config.html', **initialData)

@socketio.on('zero odo', namespace='/test')
def zero_odo(message):
    global globalData
    globalData['bZeroODO'] = True


@socketio.on('start', namespace='/test')
def start(message):
    global globalData
    globalData['bStart']   = True
    globalData['bAtWypt']  = False


@socketio.on('load waypoints', namespace='/test')
def load_waypoints(message):
    global globalData
    globalData['wypts'] = pandas.read_csv('ss.txt',sep='+',header=None).as_matrix()
    globalData['wypts'] = np.nan_to_num(globalData['wypts']).tolist()
    for i in range(0,len(globalData['wypts'])):
        globalData['wypts'][i][0] = globalData['wypts'][i][0] * 1000 # Transform to metres
    if globalData['wypts'][0][1] != 0:
        globalData['wypts'][0][1] = globalData['wypts'][0][1] / 3.6
    else:
        globalData['wypts'][0][1] = 50.0 / 3.6 # Default average if missing on first row
    currentAve = globalData['wypts'][0][1]
    for i in range(1,len(globalData['wypts'])):
        if globalData['wypts'][i][1] == 0:
            globalData['wypts'][i][1] = currentAve
        else:
            globalData['wypts'][i][1] = globalData['wypts'][i][1] / 3.6
            currentAve = globalData['wypts'][i][1]
    print(globalData['wypts'])


@socketio.on('set waypoint', namespace='/test')
def set_waypoint(message):
    global globalData
    globalData['bAtWypt'] = True
    globalData['numWypt']  = int(message)


@socketio.on('set time', namespace='/test')
def set_time(message):
    print(message)
    HH = message[0:2]
    MM = message[2:4]
    SS = message[4:6]
    print('sudo date -s "Mon Mar 14 ' + HH + ':' + MM + ':' + SS  + ' UTC 2016"')
    os.system('sudo date -s "Mon Mar 14 ' + HH + ':' + MM + ':' + SS  + ' UTC 2016"')


@socketio.on('my event', namespace='/test')
def test_message(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my response',
         {'data': message['data'], 'count': session['receive_count']})


@socketio.on('my broadcast event', namespace='/test')
def test_broadcast_message(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my response',
         {'data': message['data'], 'count': session['receive_count']},
         broadcast=True)


@socketio.on('connect', namespace='/test')
def test_connect():
    emit('my response', {'data': 'Connected', 'count': 0})


@socketio.on('disconnect', namespace='/test')
def test_disconnect():
    print('Client disconnected', request.sid)


if __name__ == '__main__':
    socketio.run(tsdapp, host= '0.0.0.0', port=8000, debug=True)
