#!/usr/bin/env python3
"""PC side of the AAI03 robot MQTT protocol (Buddy 1).

Decodes the compact binary frames the robot publishes and builds the binary
command frames it accepts.  The bit layouts mirror app_program/buddy1_comms/
tlm_codec.h and the owner headers named below; fields are packed
least-significant bit first with no padding (Change Requirement 1).

Usage:
  python3 tools/tlm_decode.py decode 10 07 ...        decode one payload (hex)
  python3 tools/tlm_decode.py monitor --broker 192.168.1.10
  python3 tools/tlm_decode.py send start --broker 192.168.1.10
  python3 tools/tlm_decode.py send map B uturn --version 2 --broker ...

monitor and send need the paho-mqtt package (pip install paho-mqtt).
"""

import argparse
import sys
import threading

TOPIC_ROOT = 'inf2004/aai03/'
VERSION = 1

VEHICLE_STATES = ['INITIALISING', 'READY', 'CALIBRATING', 'LINE_FOLLOWING',
                  'JUNCTION_ACTION', 'OBSTACLE_SCAN', 'BYPASS',
                  'LINE_REACQUIRE', 'REVERSE_RETRY', 'SAFE_STOP',
                  'MISSION_COMPLETE']
ACTIONS = ['LEFT', 'RIGHT', 'STRAIGHT', 'UTURN']
MOTION_MODES = ['IDLE', 'FOLLOWING', 'MOVING', 'TURNING', 'BRAKING',
                'STALLED']
LINE_STATES = ['CALIBRATING', 'TRACKING', 'JUNCTION', 'LOST', 'SEARCHING']
BARCODE_STATES = ['IDLE', 'CAPTURING', 'DECODING', 'REPORT', 'REJECT']
IMU_CLASSES = ['STATIONARY', 'CRUISING', 'ACCELERATING', 'DECELERATING',
               'TURNING', 'CLIMBING', 'DESCENDING', 'IMPACT']
HUMP_STATES = ['LEVEL', 'CLIMBING', 'DESCENDING']
OBSTACLE_STATES = ['MONITOR', 'DETECTED', 'COARSE_SCAN', 'FINE_SCAN',
                   'PROFILE', 'ECHO_FAULT']
PLANS = ['CONTINUE', 'STOP', 'PASS_LEFT', 'PASS_RIGHT', 'REVERSE']
LINKS = ['WIFI_INIT', 'WIFI_CONNECTING', 'MQTT_CONNECTING', 'ONLINE',
         'LOST', 'RADIO_FAULT', 'DISABLED']
FAULTS = ['MOTOR_STALL', 'ENCODER', 'IMU', 'NO_ECHO', 'WIFI_LOSS',
          'IR_CALIB', 'BARCODE_OVF', 'MQTT_LOSS']
REJECTS = ['OVERFLOW', 'COUNT', 'RATIO', 'NO_MATCH', 'FRAME', 'START']
EVENTS = ['BARCODE', 'DECODE_FAIL', 'HUMP', 'IMPACT', 'OBSTACLE', 'PROFILE',
          'STATE', 'FAULT', 'JUNCTION', 'MAP_ACK', 'LINE', 'TUNE', 'LIFT']
COMMANDS = {'start': 0, 'stop': 1, 'reset': 2, 'summary': 3, 'map': 4,
            'scan': 6}
TUNE_COMMAND = 5
# Live-tunable settings, in app_program/platform/tune.h order (= MQTT id).
TUNE_PARAMS = ['motor_max', 'speed_follow', 'speed_slow', 'speed_turn',
               'steer_kp', 'steer_kd', 'steer_dir', 'enc_um_per_edge',
               'track_mm', 'reverse_mm', 'turn_right_pct', 'turn_left_pct',
               'obs_beam_deg', 'obs_detect_mm']

# (field name, width, kind) per status word, in sys_status_word_t order.
# kind: 'u' unsigned, 's' excess-K signed, or a list of names.
STATUS_WORDS = [
    ('vehicle', [('state', 4, VEHICLE_STATES), ('latched_action', 2, ACTIONS),
                 ('latched', 1, 'u'), ('map_version', 4, 'u'),
                 ('junctions', 6, 'u'), ('bypasses', 4, 'u'),
                 ('retries', 2, 'u')]),
    ('motion', [('mode', 3, MOTION_MODES), ('left_mm_s', 11, 's'),
                ('right_mm_s', 11, 's'), ('stalled', 1, 'u')]),
    ('enc_counts', [('left', 16, 'u'), ('right', 16, 'u')]),
    ('odometry', [('distance_cm', 16, 'u')]),
    ('line', [('offset', 11, 's'), ('centre_raw', 12, 'u'),
              ('left_black', 1, 'u'), ('state', 3, LINE_STATES),
              ('stamp', 5, 'u')]),
    ('barcode', [('letter', 5, 'letter'), ('reversed', 1, 'u'),
                 ('seq', 2, 'u'), ('reads', 8, 'u'), ('rejects', 8, 'u'),
                 ('state', 3, BARCODE_STATES)]),
    ('imu', [('pitch_half_deg', 8, 's'), ('heading_deg', 9, 'u'),
             ('class', 3, IMU_CLASSES), ('hump', 2, HUMP_STATES),
             ('valid', 1, 'u'), ('impacts', 4, 'u')]),
    ('hump', [('last_mm', 8, 'u'), ('max_mm', 8, 'u'), ('count', 6, 'u')]),
    ('obstacle', [('front_cm', 8, 'u'), ('state', 3, OBSTACLE_STATES),
                  ('last_plan', 3, PLANS), ('profiles', 6, 'u'),
                  ('echo_faults', 4, 'u')]),
    ('comms', [('link', 3, LINKS), ('reconnects', 8, 'u'),
               ('dropped', 8, 'u')]),
    ('health', [('missed_periods', 8, 'u'), ('kernel_errors', 8, 'u'),
                ('faults', 8, 'faults')]),
]

EVENT_LAYOUTS = {
    'BARCODE': [('letter', 5, 'letter'), ('reversed', 1, 'u'),
                ('seq', 2, 'u')],
    'DECODE_FAIL': [('reason', 3, REJECTS), ('elements', 6, 'u')],
    'HUMP': [('height_mm', 8, 'u'), ('max_mm', 8, 'u')],
    'IMPACT': [('peak_16mg', 8, 'u')],
    'OBSTACLE': [('distance_cm', 8, 'u')],
    'PROFILE': [('plan', 3, PLANS), ('width_cm', 8, 'u'),
                ('distance_cm', 8, 'u')],
    'STATE': [('new', 4, VEHICLE_STATES), ('previous', 4, VEHICLE_STATES)],
    'FAULT': [('faults', 8, 'faults')],
    'JUNCTION': [('action', 2, ACTIONS)],
    'MAP_ACK': [('letter', 5, 'letter'), ('action', 2, ACTIONS),
                ('version', 4, 'u'), ('applied', 1, 'u')],
    'LINE': [('lost', 1, 'u')],
    'TUNE': [('setting', 4, TUNE_PARAMS), ('value', 16, 's')],
}


class BitReader:
    """Reads fields least-significant bit first from a byte string."""

    def __init__(self, data):
        self.data = data
        self.pos = 0

    def get(self, width):
        value = 0
        for bit in range(width):
            index = self.pos + bit
            if index // 8 >= len(self.data):
                raise ValueError('frame too short')
            if (self.data[index // 8] >> (index % 8)) & 1:
                value |= 1 << bit
        self.pos += width
        return value


def interpret(value, width, kind):
    """Turn a raw field into a readable value."""
    if kind == 's':
        return value - (1 << (width - 1))
    if kind == 'letter':
        return chr(ord('A') + value) if value < 26 else '?'
    if kind == 'faults':
        return [name for bit, name in enumerate(FAULTS) if value & (1 << bit)]
    if isinstance(kind, list):
        return kind[value] if value < len(kind) else value
    return value


def read_fields(reader, layout):
    """Read a list of (name, width, kind) fields."""
    return {name: interpret(reader.get(width), width, kind)
            for name, width, kind in layout}


def decode(payload):
    """Decode one robot payload into a dictionary."""
    reader = BitReader(payload)
    frame_type = reader.get(4)
    version = reader.get(4)
    if frame_type == 5:
        return {'frame': 'OFFLINE (last will)'}
    if version != VERSION:
        return {'frame': 'unknown version %d' % version}
    if frame_type == 0:
        result = {'frame': 'TELEMETRY', 'seq': reader.get(8)}
        for name, layout in STATUS_WORDS:
            result[name] = read_fields(reader, layout)
        return result
    if frame_type == 1:
        return {'frame': 'HEARTBEAT', 'seq': reader.get(8),
                'uptime_s': reader.get(16),
                'state': interpret(reader.get(4), 4, VEHICLE_STATES),
                'faults': interpret(reader.get(8), 8, 'faults')}
    if frame_type == 2:
        seq = reader.get(8)
        kind = interpret(reader.get(4), 4, EVENTS)
        data = BitReader(reader.get(28).to_bytes(4, 'little'))
        fields = read_fields(data, EVENT_LAYOUTS.get(kind, []))
        return {'frame': 'EVENT', 'seq': seq, 'event': kind, **fields}
    if frame_type == 3:
        return {'frame': 'SUMMARY', 'max_hump_mm': reader.get(8),
                'humps': reader.get(6), 'barcodes': reader.get(8),
                'rejects': reader.get(8), 'obstacles': reader.get(6),
                'distance_cm': reader.get(16), 'run_time_s': reader.get(16),
                'faults': interpret(reader.get(8), 8, 'faults')}
    if frame_type == 4:
        command = reader.get(4)
        names = {value: key for key, value in COMMANDS.items()}
        names[TUNE_COMMAND] = 'tune'
        return {'frame': 'ACK', 'command': names.get(command, command),
                'accepted': bool(reader.get(1))}
    return {'frame': 'unknown type %d' % frame_type}


# --pretty display: ANSI colours and plain-language reject reasons.
GREEN, RED, YELLOW, CYAN, DIM, BOLD, RESET = (
    '\033[92m', '\033[91m', '\033[93m', '\033[96m', '\033[2m', '\033[1m',
    '\033[0m')
REJECT_HINTS = {
    'OVERFLOW': 'too many edges (electrical noise?)',
    'COUNT': 'missed bars: thin bars too thin, sensor too high, or scan cut '
             'short',
    'RATIO': 'wide and narrow bars not clearly different (speed changed?)',
    'NO_MATCH': 'bar pattern is not a Code 39 character',
    'FRAME': 'no * stop character, or a letter outside A..Z',
    'START': 'no * start character found',
}
FRAME_ELEMENTS = 29

# Humps already printed, from the telemetry hump counter.  A HUMP event can
# be dropped on the way, so the counter in each telemetry frame is the
# backstop: whichever arrives first prints the peak height, once.
_humps = {'printed': None, 'from_status': None}
HUMP_DUPLICATE_S = 3.0


def hump_line(stamp, height_mm, max_mm, source=''):
    """The PEAK HEIGHT line for one completed hump.

    max_mm is not shown: lifting the robot by hand also reads as a hump,
    so a run maximum is not trustworthy.  Only the latest hump is kept.
    """
    del max_mm  # Deliberately not shown (see above).
    return (stamp + GREEN + BOLD + '^^^ HUMP  PEAK HEIGHT = %.1f cm%s'
            + RESET) % (height_mm / 10.0, source)


def pretty(frame):
    """One readable line (or lines) for a decoded frame, or None to skip."""
    import time

    stamp = DIM + time.strftime('%H:%M:%S') + RESET + '  '
    kind = frame.get('frame')
    if kind == 'TELEMETRY':
        veh, mot, line = frame['vehicle'], frame['motion'], frame['line']
        bar, obs, imu = frame['barcode'], frame['obstacle'], frame['imu']
        hump = frame['hump']
        faults = frame['health']['faults']
        last = bar['letter'] if bar['reads'] else '-'
        tilt = ('tilt %+5.1f deg %-12s' % (imu['pitch_half_deg'] / 2.0,
                                          imu['class'])
                if imu['valid'] else 'tilt   n/a (IMU not ready)')
        status = (stamp + '%-16s wheels L%4d R%4d mm/s  line %-9s centre %4d'
                  '  %s  humps %d%s  barcode %d ok / %d rej (last %s)  '
                  'obstacle ahead %3dcm  %s'
                  % (veh['state'], mot['left_mm_s'], mot['right_mm_s'],
                     line['state'], line['centre_raw'], tilt, hump['count'],
                     (GREEN + ' (last %.1f cm)' % (hump['last_mm'] / 10.0)
                      + RESET) if hump['count'] else '',
                     bar['reads'], bar['rejects'], last, obs['front_cm'],
                     (RED + 'faults: ' + ', '.join(faults) + RESET) if faults
                     else (GREEN + 'no faults' + RESET)))
        lines = []
        if _humps['printed'] is None or hump['count'] < _humps['printed']:
            _humps['printed'] = hump['count']  # Start-up or robot reset.
        elif hump['count'] > _humps['printed']:
            # The HUMP event was lost: print it from the status word.
            lines.append(hump_line(stamp, hump['last_mm'], hump['max_mm'],
                                   '  [from status]'))
            _humps['printed'] = hump['count']
            _humps['from_status'] = (hump['last_mm'], hump['max_mm'],
                                     time.time())
        return '\n'.join(lines + [status])
    if kind == 'EVENT':
        event = frame['event']
        if event == 'BARCODE':
            return (stamp + GREEN + BOLD + '>>> BARCODE  %s%s' + RESET) % (
                frame['letter'],
                '  (read backwards)' if frame['reversed'] else '')
        if event == 'DECODE_FAIL':
            return (stamp + RED + 'xxx BARCODE REJECTED: %s  [%d of %d '
                    'elements seen]' + RESET) % (
                        REJECT_HINTS.get(frame['reason'], frame['reason']),
                        frame['elements'], FRAME_ELEMENTS)
        if event == 'HUMP':
            recent = _humps['from_status']
            if (recent is not None and
                    recent[:2] == (frame['height_mm'], frame['max_mm']) and
                    time.time() - recent[2] < HUMP_DUPLICATE_S):
                _humps['from_status'] = None
                return None  # Already printed from the status word.
            if _humps['printed'] is not None:
                _humps['printed'] += 1  # The status counter will match.
            return hump_line(stamp, frame['height_mm'], frame['max_mm'])
        if event == 'IMPACT':
            # Peak is the departure from 1 g, in units of 16 mg.
            return (stamp + RED + BOLD + '!!! IMPACT  jolt of %.2f g beyond '
                    'normal gravity (limit 1.50 g)' + RESET) % (
                        frame['peak_16mg'] * 16 / 1000.0)
        if event == 'STATE':
            return (stamp + YELLOW + '--- STATE  %s -> %s' + RESET) % (
                frame['previous'], frame['new'])
        if event == 'FAULT':
            return (stamp + RED + BOLD + '!!! FAULT  %s' + RESET) % (
                ', '.join(frame['faults']) or 'cleared')
        details = ', '.join('%s=%s' % (key, value) for key, value
                            in frame.items()
                            if key not in ('frame', 'seq', 'event'))
        return stamp + CYAN + '--- %s  %s' % (event, details) + RESET
    if kind == 'ACK':
        colour = GREEN if frame['accepted'] else RED
        return stamp + colour + 'ACK %s %s' % (
            frame['command'],
            'accepted' if frame['accepted'] else 'REJECTED') + RESET
    if kind == 'SUMMARY':
        return (stamp + CYAN + BOLD + '=== SUMMARY  barcodes %d, rejects %d, '
                'humps %d, obstacles %d, %d cm in %d s, '
                'faults: %s' + RESET) % (
                    frame['barcodes'], frame['rejects'], frame['humps'],
                    frame['obstacles'],
                    frame['distance_cm'], frame['run_time_s'],
                    ', '.join(frame['faults']) or 'none')
    if kind == 'HEARTBEAT':
        return None  # Telemetry already shows the state every second.
    return stamp + str(frame)


# --events display: only the things that happen, one line each, with the
# details in braces.  The status frames that arrive five times a second are
# not shown; they are only used to catch an event frame that was lost.
EVENT_REPEAT_S = 3.0
PLAN_WORDS = {'CONTINUE': 'carry on', 'STOP': 'stop', 'PASS_LEFT': 'pass on '
              'the left', 'PASS_RIGHT': 'pass on the right',
              'REVERSE': 'reverse'}
_view = {'recent': {}, 'state': None, 'reads': None, 'rejects': None,
         'humps': None, 'profiles': None, 'faults': None, 'link': None,
         'latched': 0, 'reconnects': None}


def event_line(colour, title, details=None, note=''):
    """One event: time, a fixed-width title, then {Key: value, ...}."""
    import time

    body = ''
    if details:
        body = '{' + ', '.join('%s: %s' % pair for pair in details) + '}'
    return (DIM + time.strftime('%H:%M:%S') + RESET + '  ' + colour + BOLD
            + '%-26s' % title + RESET + colour + body + note + RESET)


def is_repeat(key):
    """True if this same event was shown in the last few seconds."""
    import time

    now = time.time()
    last = _view['recent'].get(key)
    _view['recent'][key] = now
    return last is not None and (now - last) < EVENT_REPEAT_S


def events_from_status(frame):
    """Events worked out from a status frame, for lost event frames."""
    lines = []
    note = '  [from status]'
    veh, bar, hump = frame['vehicle'], frame['barcode'], frame['hump']
    obs, comms = frame['obstacle'], frame['comms']
    faults = tuple(frame['health']['faults'])

    if _view['state'] != veh['state']:
        if _view['state'] is not None and not is_repeat(('state',
                                                         veh['state'])):
            lines.append(state_line(_view['state'], veh['state'], note))
        _view['state'] = veh['state']
    if _view['reads'] is not None and bar['reads'] > _view['reads']:
        if not is_repeat(('barcode', bar['letter'], bar['seq'])):
            lines.append(barcode_line(bar['letter'], bar['reversed'], note))
    if _view['rejects'] is not None and bar['rejects'] > _view['rejects']:
        if not is_repeat(('reject',)):
            lines.append(event_line(RED, 'Barcode Read Failed',
                                    [('Rejected so far', bar['rejects'])],
                                    note))
    if _view['humps'] is not None and hump['count'] > _view['humps']:
        if not is_repeat(('hump', hump['last_mm'])):
            lines.append(hump_event(hump['last_mm'], note))
    if _view['profiles'] is not None and obs['profiles'] > _view['profiles']:
        if not is_repeat(('profile', obs['last_plan'])):
            lines.append(event_line(CYAN, 'Obstacle Measured',
                                    [('Plan', PLAN_WORDS.get(obs['last_plan'],
                                                             obs['last_plan'])
                                      )], note))
    if veh['latched'] and not _view['latched']:
        lines.append(event_line(GREEN, 'Instruction Queued',
                                [('At next junction', veh['latched_action'])]))
    if _view['faults'] is not None and faults != _view['faults']:
        if not is_repeat(('fault', faults)):
            lines.append(fault_line(faults, note))
    if _view['link'] is not None and (comms['link'] != _view['link'] or
                                      comms['reconnects'] !=
                                      _view['reconnects']):
        lines.append(event_line(YELLOW, 'Link',
                                [('Status', comms['link']),
                                 ('Reconnects', comms['reconnects'])]))
    _view.update(reads=bar['reads'], rejects=bar['rejects'],
                 humps=hump['count'], profiles=obs['profiles'],
                 faults=faults, link=comms['link'], latched=veh['latched'],
                 reconnects=comms['reconnects'])
    return lines


def state_line(previous, new, note=''):
    """A change of vehicle state, with the stops called out."""
    if new == 'SAFE_STOP':
        return event_line(RED, 'Robot Stopped',
                          [('Was', previous), ('Now', new)], note)
    return event_line(YELLOW, 'State Changed',
                      [('From', previous), ('To', new)], note)


def barcode_line(letter, is_reversed, note=''):
    """A decoded barcode."""
    return event_line(GREEN, 'Barcode Read Successful',
                      [('Result', letter),
                       ('Scanned', 'backwards' if is_reversed
                        else 'forwards')], note)


def hump_event(height_mm, note=''):
    """A measured hump."""
    return event_line(GREEN, 'Hump Measured',
                      [('Height', '%.1f cm' % (height_mm / 10.0))], note)


def fault_line(faults, note=''):
    """The set of active faults after a change."""
    if faults:
        return event_line(RED, 'Fault', [('Active', ' + '.join(faults))],
                          note)
    return event_line(GREEN, 'Faults Cleared', None, note)


def events_view(frame):
    """Lines to show for one decoded frame in --events mode (may be none)."""
    kind = frame.get('frame')
    if kind == 'TELEMETRY':
        return events_from_status(frame)
    if kind == 'HEARTBEAT':
        return []
    if kind == 'ACK':
        return [event_line(GREEN if frame['accepted'] else RED,
                           'Command Acknowledged',
                           [('Command', frame['command']),
                            ('Accepted', 'yes' if frame['accepted']
                             else 'NO')])]
    if kind == 'SUMMARY':
        return [event_line(CYAN, 'Run Summary',
                           [('Barcodes', frame['barcodes']),
                            ('Rejected', frame['rejects']),
                            ('Humps', frame['humps']),
                            ('Highest hump', '%.1f cm'
                             % (frame['max_hump_mm'] / 10.0)),
                            ('Obstacles', frame['obstacles']),
                            ('Distance', '%d cm' % frame['distance_cm']),
                            ('Time', '%d s' % frame['run_time_s']),
                            ('Faults', ' + '.join(frame['faults'])
                             or 'none')])]
    if kind != 'EVENT':
        return [event_line(RED, 'Robot Offline' if 'OFFLINE' in str(kind)
                           else str(kind))]

    event = frame['event']
    if event == 'BARCODE':
        if is_repeat(('barcode', frame['letter'], frame['seq'])):
            return []
        return [barcode_line(frame['letter'], frame['reversed'])]
    if event == 'DECODE_FAIL':
        is_repeat(('reject',))
        return [event_line(RED, 'Barcode Read Failed',
                           [('Reason', REJECT_HINTS.get(frame['reason'],
                                                        frame['reason'])),
                            ('Elements', '%d of %d' % (frame['elements'],
                                                       FRAME_ELEMENTS))])]
    if event == 'HUMP':
        if is_repeat(('hump', frame['height_mm'])):
            return []
        return [hump_event(frame['height_mm'])]
    if event == 'OBSTACLE':
        return [event_line(CYAN, 'Obstacle Detected',
                           [('Distance', '%d cm' % frame['distance_cm'])])]
    if event == 'PROFILE':
        is_repeat(('profile', frame['plan']))
        return [event_line(CYAN, 'Obstacle Measured',
                           [('Width', '%d cm' % frame['width_cm']),
                            ('Distance', '%d cm' % frame['distance_cm']),
                            ('Plan', PLAN_WORDS.get(frame['plan'],
                                                    frame['plan']))])]
    if event == 'STATE':
        _view['state'] = frame['new']
        if is_repeat(('state', frame['new'])):
            return []
        return [state_line(frame['previous'], frame['new'])]
    if event == 'FAULT':
        faults = tuple(frame['faults'])
        if is_repeat(('fault', faults)):
            return []
        return [fault_line(faults)]
    if event == 'JUNCTION':
        return [event_line(GREEN, 'Junction Instruction Run',
                           [('Action', frame['action'])])]
    if event == 'MAP_ACK':
        return [event_line(GREEN if frame['applied'] else RED,
                           'Barcode Mapping Changed',
                           [('Letter', frame['letter']),
                            ('Action', frame['action']),
                            ('Version', frame['version']),
                            ('Applied', 'yes' if frame['applied']
                             else 'NO')])]
    if event == 'LINE':
        return [event_line(RED, 'Line Lost') if frame['lost']
                else event_line(GREEN, 'Line Found')]
    if event == 'LIFT':
        return [event_line(RED, 'Robot Picked Up',
                           [('Motors', 'stopped')])]
    if event == 'IMPACT':
        return [event_line(YELLOW, 'Jolt',
                           [('Size', '%.2f g' % (frame['peak_16mg'] * 16
                                                 / 1000.0))])]
    if event == 'TUNE':
        return [event_line(CYAN, 'Setting Changed',
                           [(frame['setting'], frame['value'])])]
    details = [(key, value) for key, value in frame.items()
               if key not in ('frame', 'seq', 'event')]
    return [event_line(CYAN, str(event), details)]


def build_command(name, letter=None, action=None, map_version=0):
    """Build a binary command payload."""
    header = 0x08 | (VERSION << 4)
    body = COMMANDS[name] | ((map_version & 0x0F) << 4)
    payload = bytes([header, body])
    if name == 'map':
        index = ord(letter.upper()) - ord('A')
        if not 0 <= index < 26:
            raise ValueError('letter must be A..Z')
        payload += bytes([index | (ACTIONS.index(action.upper()) << 5)])
    return payload


def build_tune(setting, value):
    """Build a TUNE command payload: setting name and signed 16-bit value."""
    index = TUNE_PARAMS.index(setting)
    if not -32768 <= value <= 32767:
        raise ValueError('value must fit in 16 bits')
    header = 0x08 | (VERSION << 4)
    return bytes([header, TUNE_COMMAND | (index << 4)]) + \
        (value & 0xFFFF).to_bytes(2, 'little')


def run_mqtt(args):
    """monitor or send over MQTT (needs paho-mqtt)."""
    import paho.mqtt.client as mqtt

    # paho-mqtt 2.x requires the callback API version; 1.x has no such
    # argument.  Version 1 callbacks match on_message below.
    if hasattr(mqtt, 'CallbackAPIVersion'):
        import warnings
        with warnings.catch_warnings():
            # VERSION1 is deprecated in paho 2.x but still works.
            warnings.simplefilter('ignore', DeprecationWarning)
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    else:
        client = mqtt.Client()
    if args.mode in ('send', 'tune'):
        connected = threading.Event()
        connack = {}

        def on_connect(_client, _userdata, _flags, rc):
            connack['rc'] = rc
            connected.set()

        client.on_connect = on_connect
        client.connect(args.broker, args.port, 10)
        client.loop_start()
        if not connected.wait(5.0):
            client.loop_stop()
            print('no reply from broker %s' % args.broker, file=sys.stderr)
            return 1
        if connack['rc'] != 0:
            client.loop_stop()
            print('broker %s refused the connection: %s'
                  % (args.broker, mqtt.connack_string(connack['rc'])),
                  file=sys.stderr)
            return 1
        if args.mode == 'tune':
            payload = build_tune(args.setting, args.value)
            args.command = '%s=%d' % (args.setting, args.value)
        else:
            payload = build_command(args.command, args.letter, args.action,
                                    args.version)
        client.publish(TOPIC_ROOT + 'cmd', payload, qos=0).wait_for_publish(5.0)
        client.disconnect()
        client.loop_stop()
        print('sent %s: %s' % (args.command, payload.hex(' ')))
        return 0

    if (args.pretty or args.events) and sys.platform == 'win32':
        import os
        os.system('')  # Turns on ANSI colour handling in the Windows console.

    log_file = open(args.log, 'a') if args.log else None

    def on_message(_client, _userdata, message):
        if message.topic.endswith('/dbg'):
            # Diagnostic build only: raw record chunks, kept as hex.
            if log_file is not None:
                import time
                log_file.write('%.3f %s HEX %s\n' % (time.time(), message.topic,
                                                    message.payload.hex()))
                log_file.flush()
            return
        try:
            if log_file is not None:
                import time
                log_file.write('%.3f %s %r\n' % (time.time(), message.topic,
                                                 decode(message.payload)))
                log_file.flush()
            if args.events and message.topic.endswith('/cmd'):
                pass  # A command on its way to the robot, not robot data.
            elif args.events:
                for line in events_view(decode(message.payload)):
                    print(line, flush=True)
            elif args.pretty:
                line = pretty(decode(message.payload))
                if line is not None:
                    print(line, flush=True)
            else:
                print(message.topic, decode(message.payload), flush=True)
        except ValueError as error:
            print(message.topic, 'undecodable:', error,
                  message.payload.hex(' '))

    def on_connect(mqtt_client, _userdata, _flags, rc):
        # Subscribe here, not after connect(): paho reconnects on its own
        # after a dropped link but does not restore subscriptions made
        # outside this callback, so the monitor would go silent for good.
        print('connected to %s: %s' % (args.broker, mqtt.connack_string(rc)),
              flush=True)
        if rc == 0:
            mqtt_client.subscribe(TOPIC_ROOT + '#')

    def on_disconnect(_client, _userdata, rc):
        print('disconnected from broker (rc=%s), reconnecting' % rc,
              file=sys.stderr, flush=True)

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.connect(args.broker, args.port, 10)
    client.loop_forever(retry_first_connection=True)
    return 0


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='mode', required=True)
    dec = sub.add_parser('decode')
    dec.add_argument('hex', nargs='+')
    mon = sub.add_parser('monitor')
    mon.add_argument('--pretty', action='store_true',
                     help='one coloured line per frame, events highlighted')
    mon.add_argument('--events', action='store_true',
                     help='only what happens (barcode read, hump, obstacle, '
                          'stop ...), one line each; no status polls')
    mon.add_argument('--log', metavar='FILE',
                     help='also append every decoded frame to FILE')
    snd = sub.add_parser('send')
    snd.add_argument('command', choices=sorted(COMMANDS))
    snd.add_argument('letter', nargs='?')
    snd.add_argument('action', nargs='?', choices=[a.lower() for a in ACTIONS]
                     + ACTIONS)
    snd.add_argument('--version', type=int, default=0)
    tun = sub.add_parser('tune', help='change a setting live; lost on reset')
    tun.add_argument('setting', choices=TUNE_PARAMS)
    tun.add_argument('value', type=int)
    for sub_parser in (mon, snd, tun):
        sub_parser.add_argument('--broker', required=True)
        sub_parser.add_argument('--port', type=int, default=1883)
    args = parser.parse_args(argv[1:])
    if args.mode == 'decode':
        print(decode(bytes.fromhex(''.join(args.hex))))
        return 0
    return run_mqtt(args)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
