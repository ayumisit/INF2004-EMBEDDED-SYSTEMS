#!/usr/bin/env python3
"""AAI03 robot live dashboard (MQTT).

One screen that updates in place: the robot's state, safety, wheels, line,
barcode, humps, obstacle and IMU at the top, and the latest events (newest
first, colour coded) underneath.  Nothing scrolls, so nothing spams.

Usage:
  python tools/robot_dashboard.py                     (broker 172.20.10.3)
  python tools/robot_dashboard.py --broker 192.168.1.10

Keys while it runs:  S start   X stop   R reset   U summary   Q quit
  W  scan the obstacle in front now and show its width (robot stopped)
  M  send the barcode map: type the letters for FORWARD, U-TURN, LEFT and
     RIGHT (Backspace undoes, Esc cancels); they replace any earlier map

Every decoded frame is also written to tools/logs/ for checking later.
Needs paho-mqtt (pip install paho-mqtt).  Decoding is shared with
tlm_decode.py, so both always agree with the firmware.
"""

import argparse
import collections
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tlm_decode as tlm  # noqa: E402  (same folder)

DEFAULT_BROKER = '172.20.10.3'
REFRESH_S = 0.25
SILENT_S = 3.0          # No message for this long: robot shown OFFLINE.
EVENT_ROWS = 14
WIDTH = 78

# The four movements of the brief, in the order the M prompt asks for
# them, as (prompt word, MQTT action name).
MAP_MOVES = [('FORWARD', 'straight'), ('U-TURN', 'uturn'), ('LEFT', 'left'),
             ('RIGHT', 'right')]
ACTION_WORDS = {'STRAIGHT': 'forward', 'UTURN': 'U-turn', 'LEFT': 'left',
                'RIGHT': 'right'}
KEY_ESC, KEY_BACKSPACE = chr(27), chr(8)

# Colours (ANSI).
R, G, Y, B, M, C, W = ('\033[91m', '\033[92m', '\033[93m', '\033[94m',
                       '\033[95m', '\033[96m', '\033[97m')
DIM, BOLD, RST = '\033[2m', '\033[1m', '\033[0m'
REV = '\033[7m'

STATE_COLOUR = {
    'INITIALISING': Y, 'READY': C, 'CALIBRATING': Y, 'LINE_FOLLOWING': G,
    'JUNCTION_ACTION': B, 'OBSTACLE_SCAN': M, 'BYPASS': M,
    'LINE_REACQUIRE': Y, 'REVERSE_RETRY': Y, 'SAFE_STOP': R,
    'MISSION_COMPLETE': G,
}
STATE_WORDS = {
    'INITIALISING': 'starting up', 'READY': 'ready - waiting for START',
    'CALIBRATING': 'calibrating sensors', 'LINE_FOLLOWING': 'following line',
    'JUNCTION_ACTION': 'turning at junction',
    'OBSTACLE_SCAN': 'scanning obstacle', 'BYPASS': 'driving round obstacle',
    'LINE_REACQUIRE': 'looking for line', 'REVERSE_RETRY': 'reversing',
    'SAFE_STOP': 'STOPPED (safety)', 'MISSION_COMPLETE': 'mission complete',
}
LINK_COLOUR = {'ONLINE': G, 'LOST': R, 'RADIO_FAULT': R}


class Board:
    """Latest robot data, shared between the MQTT thread and the screen."""

    def __init__(self):
        self.lock = threading.Lock()
        self.telemetry = None
        self.heartbeat = None
        self.summary = None
        self.profile = None     # Last obstacle scan (width, distance, plan).
        self.last_rx = None
        self.broker_ok = False
        self.broker_msg = 'connecting...'
        self.events = collections.deque(maxlen=EVENT_ROWS)
        self.frames = 0
        # Barcode map: version -> {letter: action}, from MAP_ACK events.
        self.maps = {}
        self.map_version = None   # From the robot's status word.
        self.prompt = None        # Letters typed so far while M is open.
        self.sent_map = []        # (move, letter) pairs of the last M map.


def bar(value, full, width, colour):
    """A horizontal bar for a value from 0 to full."""
    filled = max(0, min(width, int(round(abs(value) * width / float(full)))))
    return colour + '#' * filled + DIM + '.' * (width - filled) + RST


def gauge(offset, limit=1000, width=21):
    """Line offset gauge: | marks the centre, O the line position."""
    half = width // 2
    pos = half + int(round(max(-limit, min(limit, offset)) * half
                           / float(limit)))
    cells = ['-'] * width
    cells[half] = '|'
    cells[pos] = 'O'
    text = ''.join(cells)
    return DIM + '[' + RST + text.replace('O', BOLD + Y + 'O' + RST) + \
        DIM + ']' + RST


def row(label, text):
    return ' ' + BOLD + W + '%-10s' % label + RST + ' ' + text


def render(board, broker):
    """Build the whole screen as a list of lines."""
    with board.lock:
        tel, hb, summ = board.telemetry, board.heartbeat, board.summary
        profile = board.profile
        last_rx, events = board.last_rx, list(board.events)
        prompt = None if board.prompt is None else list(board.prompt)
        sent_map = list(board.sent_map)
        version = board.map_version
        current = dict(board.maps.get(version, {}))
        broker_ok, broker_msg = board.broker_ok, board.broker_msg
        frames = board.frames

    now = time.time()
    lines = []
    title = ' AAI03 ROBOT DASHBOARD '
    lines.append(REV + BOLD + title + ' ' * (WIDTH - len(title)) + RST)

    if not broker_ok:
        link = R + BOLD + 'BROKER NOT CONNECTED' + RST + DIM + '  (%s)' \
            % broker_msg + RST
    elif last_rx is None:
        link = Y + BOLD + 'WAITING FOR ROBOT' + RST + DIM + \
            '  (power it on; it joins the hotspot in ~10 s)' + RST
    elif now - last_rx > SILENT_S:
        link = R + BOLD + 'ROBOT OFFLINE' + RST + DIM + \
            '  (nothing for %d s)' % (now - last_rx) + RST
    else:
        link = G + BOLD + 'ROBOT ONLINE' + RST + DIM + \
            '  (%d frames)' % frames + RST
    lines.append(row('Link', link + DIM + '   broker %s' % broker + RST))
    lines.append(DIM + '-' * WIDTH + RST)

    if tel is None:
        lines.append('')
        lines.append(DIM + '   No status received yet.' + RST)
        lines += [''] * 11
    else:
        veh, mot, line = tel['vehicle'], tel['motion'], tel['line']
        bc, imu, hump = tel['barcode'], tel['imu'], tel['hump']
        obs, comms = tel['obstacle'], tel['comms']
        faults = tel['health']['faults']
        state = veh['state']
        colour = STATE_COLOUR.get(state, W)
        uptime = (' ' * 3 + DIM + 'up %d s' % hb['uptime_s'] + RST) if hb \
            else ''
        lines.append(row('STATE', colour + BOLD + '%-17s' % state + RST + ' '
                         + colour + STATE_WORDS.get(state, '') + RST
                         + uptime))

        imu_ok = imu['valid'] and 'IMU' not in faults
        if imu_ok:
            safety = G + BOLD + 'LIFT STOP ARMED' + RST + DIM + \
                '  (tip it when picking up)' + RST
        else:
            safety = R + BOLD + REV + ' LIFT STOP NOT WORKING ' + RST + R + \
                '  IMU fault: switch off, plug in USB, switch on, unplug' + RST
        lines.append(row('SAFETY', safety))
        lines.append(row('FAULTS', (R + BOLD + ' + '.join(faults) + RST)
                         if faults else G + 'none' + RST))

        lines.append(row('WHEELS', 'L %4d %s   R %4d %s  %s' % (
            mot['left_mm_s'], bar(mot['left_mm_s'], 150, 12, C),
            mot['right_mm_s'], bar(mot['right_mm_s'], 150, 12, C),
            (R + BOLD + 'STALLED' + RST) if mot['stalled']
            else DIM + str(mot['mode']) + RST)))
        lstate = line['state']
        lcol = G if lstate == 'TRACKING' else (R if lstate == 'LOST' else Y)
        lines.append(row('LINE', lcol + '%-10s' % lstate + RST + ' '
                         + gauge(line['offset']) + DIM
                         + '  sensor %4d%s' % (line['centre_raw'],
                                               '  left sees line'
                                               if line['left_black'] else '')
                         + RST))
        last = bc['letter'] if bc['reads'] else '-'
        lines.append(row('BARCODE', 'last ' + G + BOLD + '%s' % last + RST
                         + '   read ' + G + '%d' % bc['reads'] + RST
                         + '   failed ' + (R if bc['rejects'] else DIM)
                         + '%d' % bc['rejects'] + RST
                         + DIM + '   (%s)' % bc['state'] + RST))
        if current:
            order = ['STRAIGHT', 'UTURN', 'LEFT', 'RIGHT']
            pairs = sorted(current.items(),
                           key=lambda item: order.index(item[1])
                           if item[1] in order else 9)
            lines.append(row('MAP', G + BOLD + '   '.join(
                '%s = %s' % (letter, ACTION_WORDS.get(action, action))
                for letter, action in pairs) + RST + DIM
                + '   (version %d, others go straight)' % version + RST))
        elif version:
            lines.append(row('MAP', Y + 'version %d is on the robot (sent '
                             'before this dashboard opened)' % version
                             + RST + DIM + '   press M to send again' + RST))
        else:
            lines.append(row('MAP', R + BOLD + 'NO MAP SENT' + RST + R
                             + '  every barcode goes straight - press M' + RST))
        lines.append(row('HUMPS', 'count ' + G + BOLD + '%d' % hump['count']
                         + RST + '   last ' + G + '%.1f cm' %
                         (hump['last_mm'] / 10.0) + RST
                         + DIM + '   (%s)' % imu['hump'] + RST))
        front = obs['front_cm']
        fcol = R if front and front < 20 else (Y if front and front < 40
                                               else G)
        lines.append(row('OBSTACLE', 'ahead ' + fcol + BOLD + '%3d cm' % front
                         + RST + '   ' + DIM + '%s, %d measured'
                         % (obs['state'], obs['profiles']) + RST))
        if profile is None:
            lines.append(row('', DIM + 'width: not scanned yet - the robot '
                             'scans when it meets an obstacle on a run, or '
                             'press ' + RST + BOLD + 'W' + RST + DIM
                             + ' while stopped' + RST))
        else:
            lines.append(row('', 'WIDTH ' + M + BOLD + REV + ' %d cm '
                             % profile['width_cm'] + RST + '   at '
                             + M + '%d cm' % profile['distance_cm'] + RST
                             + '   plan ' + M + BOLD
                             + tlm.PLAN_WORDS.get(profile['plan'],
                                                  str(profile['plan']))
                             + RST + DIM + '   (scanned %s)'
                             % profile['time'] + RST))
        tilt = ('%5.1f deg' % (imu['pitch_half_deg'] / 2.0)) if imu['valid'] \
            else ' n/a'
        lines.append(row('IMU', 'tilt ' + C + tilt + RST + '   ' + DIM
                         + '%s, heading %d deg, jolts %d'
                         % (imu['class'], imu['heading_deg'], imu['impacts'])
                         + RST))
        lines.append(row('TRIP', 'junctions %d   bypasses %d   distance %d cm'
                         % (veh['junctions'], veh['bypasses'],
                            tel['odometry']['distance_cm'])
                         + (('   next turn ' + B + str(veh['latched_action']) + RST)
                            if veh['latched'] else '')))
        lines.append(row('WIFI', LINK_COLOUR.get(comms['link'], Y)
                         + str(comms['link']) + RST + DIM
                         + '   reconnects %d, dropped %d'
                         % (comms['reconnects'], comms['dropped']) + RST))
        lines.append('')

    if summ is not None:
        lines.append(row('SUMMARY', C + 'barcodes %d, failed %d, humps %d '
                         '(max %.1f cm), obstacles %d, %d cm in %d s' % (
                             summ['barcodes'], summ['rejects'], summ['humps'],
                             summ['max_hump_mm'] / 10.0, summ['obstacles'],
                             summ['distance_cm'], summ['run_time_s']) + RST))
    if summ is not None or sent_map:
        lines.append(row('SUMMARY' if summ is None else '',
                         'barcode map sent over MQTT: ' + (
                             (G + BOLD + '   '.join(
                                 '%s = %s' % (letter, word.lower())
                                 for word, letter in sent_map) + RST)
                             if sent_map else R + 'none' + RST)))

    if prompt is not None:
        asked = ', '.join('%s = %s' % (MAP_MOVES[idx][0], letter)
                          for idx, letter in enumerate(prompt))
        lines.append(REV + Y + BOLD + ' MAP: type the letter for %s '
                     % MAP_MOVES[len(prompt)][0] + RST + '  ' + asked
                     + DIM + '   (Backspace undo, Esc cancel)' + RST)
    lines.append(REV + BOLD + ' EVENTS (newest at top) ' + ' ' * (WIDTH - 24)
                 + RST)
    for text in reversed(events):
        lines.append(' ' + text)
    lines += [''] * (EVENT_ROWS - len(events))
    lines.append(DIM + '-' * WIDTH + RST)
    lines.append(' ' + BOLD + 'S' + RST + ' start   ' + BOLD + 'X' + RST
                 + ' stop   ' + BOLD + 'R' + RST + ' reset   ' + BOLD + 'U'
                 + RST + ' summary   ' + BOLD + 'W' + RST + ' scan width   '
                 + BOLD + 'M' + RST + ' barcode map   ' + BOLD + 'Q' + RST
                 + ' quit')
    return lines


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--broker', default=DEFAULT_BROKER)
    parser.add_argument('--port', type=int, default=1883)
    args = parser.parse_args(argv[1:])

    import paho.mqtt.client as mqtt

    if sys.platform == 'win32':
        os.system('')  # Turn on ANSI colours in the Windows console.
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except AttributeError:
        pass

    board = Board()
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    log = open(os.path.join(log_dir, time.strftime('robot_%Y%m%d_%H%M%S.txt')),
               'a')

    if hasattr(mqtt, 'CallbackAPIVersion'):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', DeprecationWarning)
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    else:
        client = mqtt.Client()

    def on_connect(mqtt_client, _userdata, _flags, rc):
        with board.lock:
            board.broker_ok = (rc == 0)
            board.broker_msg = mqtt.connack_string(rc)
        if rc == 0:
            mqtt_client.subscribe(tlm.TOPIC_ROOT + '#')

    def on_disconnect(_client, _userdata, rc):
        with board.lock:
            board.broker_ok = False
            board.broker_msg = 'lost (rc=%s), retrying' % rc

    def on_message(_client, _userdata, message):
        if message.topic.endswith('/dbg') or message.topic.endswith('/cmd'):
            return
        try:
            frame = tlm.decode(message.payload)
        except ValueError:
            return
        log.write('%s %s %r\n' % (time.strftime('%H:%M:%S'), message.topic,
                                  frame))
        log.flush()
        new_events = tlm.events_view(frame)
        with board.lock:
            board.last_rx = time.time()
            board.frames += 1
            kind = frame.get('frame')
            if kind == 'TELEMETRY':
                board.telemetry = frame
                board.map_version = frame['vehicle']['map_version']
            elif kind == 'HEARTBEAT':
                board.heartbeat = frame
            elif kind == 'SUMMARY':
                board.summary = frame
            elif kind == 'EVENT' and frame.get('event') == 'MAP_ACK':
                if frame.get('applied'):
                    board.maps.setdefault(frame['version'], {})
                    chosen = board.maps[frame['version']]
                    for letter in [key for key, value in chosen.items()
                                   if value == frame['action']]:
                        del chosen[letter]  # One letter per movement.
                    chosen[frame['letter']] = frame['action']
                    board.map_version = frame['version']
            elif kind == 'EVENT' and frame.get('event') == 'PROFILE':
                board.profile = dict(frame, time=time.strftime('%H:%M:%S'))
            for text in new_events:
                board.events.append(text)

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    try:
        client.connect_async(args.broker, args.port, 10)
    except OSError as error:
        board.broker_msg = str(error)
    client.loop_start()

    keys = None
    if sys.platform == 'win32':
        import msvcrt
        keys = msvcrt

    def map_key(key):
        # One key while the M prompt is open.
        old = 0
        with board.lock:
            if key == KEY_ESC:
                board.prompt = None
            elif key == KEY_BACKSPACE:
                board.prompt = board.prompt[:-1]
            elif 'a' <= key <= 'z' and key.upper() not in board.prompt:
                board.prompt.append(key.upper())
            letters = list(board.prompt) if board.prompt else []
            b_done = len(letters) == len(MAP_MOVES)
            if b_done:
                board.prompt = None
                old = board.map_version or 0
        if b_done:
            # A new version (1..15, never 0) makes the robot clear the old
            # map first; the four letters share it.
            new = (old % 15) + 1
            shown = []
            for letter, (word, action) in zip(letters, MAP_MOVES):
                payload = tlm.build_command('map', letter, action, new)
                client.publish(tlm.TOPIC_ROOT + 'cmd', payload, qos=1)
                # The exact MQTT message, for showing what went over MQTT.
                shown.append(tlm.event_line(
                    B, 'MQTT sent', [('Topic', tlm.TOPIC_ROOT + 'cmd'),
                                     ('Bytes', payload.hex(' ')),
                                     ('Meaning', '%s = %s, version %d'
                                      % (letter, word.lower(), new))]))
            with board.lock:
                board.sent_map = [(word, letter) for letter, (word, _a)
                                  in zip(letters, MAP_MOVES)]
                for text in shown:
                    board.events.append(text)

    def send(name):
        client.publish(tlm.TOPIC_ROOT + 'cmd', tlm.build_command(name), qos=0)
        with board.lock:
            board.events.append(tlm.event_line(B, 'You sent', [('Command',
                                                                name.upper())]))

    sys.stdout.write('\033[2J\033[?25l')  # Clear screen, hide cursor.
    try:
        while True:
            screen = render(board, args.broker)
            out = '\033[H' + '\n'.join(text + '\033[K' for text in screen) \
                + '\033[J'
            sys.stdout.write(out)
            sys.stdout.flush()
            end = time.time() + REFRESH_S
            while time.time() < end:
                if keys is not None and keys.kbhit():
                    key = keys.getwch().lower()
                    with board.lock:
                        b_prompt = board.prompt is not None
                    if b_prompt:
                        map_key(key)
                        continue
                    if key == 'm':
                        with board.lock:
                            board.prompt = []
                        continue
                    if key == 'q':
                        return 0
                    command = {'s': 'start', 'x': 'stop', 'r': 'reset',
                               'u': 'summary', 'w': 'scan'}.get(key)
                    try:
                        if command:
                            send(command)
                    except Exception as error:  # Never close mid-run.
                        with board.lock:
                            board.events.append(tlm.event_line(
                                R, 'Key %r failed' % key, [('Error',
                                                            error)]))
                time.sleep(0.03)
    except KeyboardInterrupt:
        return 0
    finally:
        sys.stdout.write('\033[?25h' + RST + '\n')
        client.loop_stop()
        log.close()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
