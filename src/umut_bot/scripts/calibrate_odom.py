#!/usr/bin/env python3
"""Measure UmutBot's odometry parameters on the floor.

Two tests, both based on the raw encoder counts the Nano reports
(/encoder_feedback_1 = left wheel, /encoder_feedback_2 = right wheel):

  1. Push the robot straight along a tape measure by hand
     -> how far each wheel rolls per tick (left/right wheel radius)
  2. Turn it on the spot by a known number of full turns
     -> the effective wheel base

Pushing by hand keeps test 1 straight even if the robot drifts to one side
under its own power. The current odometry settings are only used for the
live display and the comparison; the results come from the raw counts.

Needs the Nano bridge (pico.launch or navigation.launch) running:
    rosrun umut_bot calibrate_odom.py
Copy the printed values into navigation.launch and the firmware.
"""

import math
import select
import sys
import threading
import time

import rospy
from std_msgs.msg import Int32

# Same limit as encoder_odom.py: a bigger jump between two reports isn't
# wheel motion, the Nano restarted and its counters went back to 0
MAX_TICKS_PER_REPORT = 500
# The Nano reports at 20 Hz; a longer silence means the USB link dropped
MAX_REPORT_GAP = 1.0  # s

# Where navigation.launch puts the odometry parameters
ODOM_NS = '/encoder_odometry_node/'

LINE = '-' * 72


class Settings:
    """A set of odometry parameters (the ones in use, or measured ones)."""

    def __init__(self, left_radius, right_radius, wheel_base, ticks_per_rev):
        self.radius = (left_radius, right_radius)
        self.wheel_base = wheel_base
        self.ticks_per_rev = ticks_per_rev

    @property
    def m_per_tick(self):
        return tuple(2 * math.pi * r / self.ticks_per_rev for r in self.radius)

    def wheel_travel(self, ticks):
        """Distance each wheel rolled, in m, for (left, right) ticks."""
        return ticks[0] * self.m_per_tick[0], ticks[1] * self.m_per_tick[1]

    def distance(self, ticks):
        left, right = self.wheel_travel(ticks)
        return (left + right) / 2

    def angle(self, ticks):
        """Heading change in rad (positive = left / counter-clockwise)."""
        left, right = self.wheel_travel(ticks)
        return (right - left) / self.wheel_base


def current_settings():
    """The parameters of the running odometry node, else the defaults."""
    radius = rospy.get_param(ODOM_NS + 'wheel_radius', 0.035)
    settings = Settings(rospy.get_param(ODOM_NS + 'left_wheel_radius', radius),
                        rospy.get_param(ODOM_NS + 'right_wheel_radius', radius),
                        rospy.get_param(ODOM_NS + 'wheel_base', 0.255),
                        rospy.get_param(ODOM_NS + 'ticks_per_revolution', 403))
    found = rospy.has_param(ODOM_NS + 'wheel_base')
    return settings, found


class Encoders:
    """Sums up the tick changes of both wheels and notices dropouts."""

    def __init__(self):
        self.lock = threading.Lock()
        self.last = [None, None]        # newest raw count per wheel
        self.last_time = [None, None]
        self.total = [0, 0]             # summed changes since the start
        self.dropouts = 0               # Nano restarts or gaps in the data
        rospy.Subscriber('/encoder_feedback_1', Int32, self.callback, 0)
        rospy.Subscriber('/encoder_feedback_2', Int32, self.callback, 1)

    def callback(self, msg, wheel):
        now = time.monotonic()
        with self.lock:
            if self.last[wheel] is not None:
                delta = msg.data - self.last[wheel]
                if (abs(delta) > MAX_TICKS_PER_REPORT
                        or now - self.last_time[wheel] > MAX_REPORT_GAP):
                    self.dropouts += 1
                if abs(delta) <= MAX_TICKS_PER_REPORT:
                    self.total[wheel] += delta
            self.last[wheel] = msg.data
            self.last_time[wheel] = now

    def ready(self):
        with self.lock:
            return None not in self.last

    def read(self):
        """Return ([left, right] summed ticks, number of dropouts so far)."""
        with self.lock:
            return list(self.total), self.dropouts

    def silent_for(self):
        """Seconds since the wheel that reported least recently reported."""
        with self.lock:
            if None in self.last_time:
                return float('inf')
            return time.monotonic() - min(self.last_time)


# ---------------------------------------------------------------- results

def radii_from_straight(runs, ticks_per_rev):
    """(left, right) wheel radius from (left ticks, right ticks, metres) runs."""
    distance = sum(run[2] for run in runs)
    radii = []
    for wheel in (0, 1):
        ticks_per_m = sum(abs(run[wheel]) for run in runs) / distance
        radii.append(ticks_per_rev / (2 * math.pi * ticks_per_m))
    return tuple(radii)


def wheel_base_from_turns(runs, settings):
    """Effective wheel base from (left ticks, right ticks, full turns) runs.

    The odometry computes the heading change as (s_right - s_left) / wheel_base,
    so the wheel base is the travel difference of the wheels per radian turned.
    """
    travel_diff = 0.0
    angle = 0.0
    for left, right, turns in runs:
        s_left, s_right = settings.wheel_travel((left, right))
        travel_diff += abs(s_right - s_left)
        angle += 2 * math.pi * turns
    return travel_diff / angle


def percent_off(value, reference):
    return 100.0 * (value - reference) / reference


# ---------------------------------------------------------------- terminal

def wait_enter(status=None):
    """Wait for Enter and return what was typed (lower case).

    status() returns a line that is redrawn until then (only in a terminal).
    """
    if status is None or not sys.stdin.isatty():
        return input().strip().lower()
    while True:
        sys.stdout.write('\r    ' + status() + '\033[K')
        sys.stdout.flush()
        ready, _, _ = select.select([sys.stdin], [], [], 0.2)
        if ready:
            return sys.stdin.readline().strip().lower()


def ask_number(prompt, maximum=None, too_big=''):
    while True:
        text = input(prompt).strip().replace(',', '.')
        try:
            value = float(text)
        except ValueError:
            print('    Please type a number, e.g. 2 or 1.5')
            continue
        if value <= 0:
            print('    Must be more than 0')
        elif maximum is not None and value > maximum:
            print('    ' + too_big)
        else:
            return value


def ask_yes(prompt):
    return input(prompt).strip().lower().startswith(('y', 'j'))


def run_ok(enc, dropouts_before):
    """False (with a message) if the encoder data dropped out during a run."""
    _, dropouts = enc.read()
    if dropouts != dropouts_before:
        print('    The encoder data dropped out during this run (Nano restarted or the')
        print('    USB link hung). Run discarded, please do it again.')
        return False
    if enc.silent_for() > MAX_REPORT_GAP:
        print('    No encoder data right now. Is the Nano bridge still running?')
        print('    Run discarded.')
        return False
    return True


# ---------------------------------------------------------------- tests

def straight_test(enc, now):
    print()
    print(LINE)
    print('TEST 1 of 2: straight line, pushed by hand')
    print(LINE)
    print('Lay a tape measure (2 m or more) on the floor. Mark where the wheel axle')
    print('is at the start, e.g. with a strip of tape next to a wheel. Then push the')
    print('robot slowly and straight along the tape, about 2 m. A wall or a long')
    print('board as a guide helps. The wheels have to roll, not slide.')
    print('Pushing backwards works too. 2-3 runs make the result more precise.')

    runs = []
    while True:
        print()
        print('Robot at the start mark? Press Enter.  (s + Enter = skip this test)')
        if wait_enter() == 's':
            return runs
        start, dropouts = enc.read()

        def status():
            ticks, _ = enc.read()
            d = (ticks[0] - start[0], ticks[1] - start[1])
            return 'left {:+6d}  right {:+6d} ticks   current odometry: {:+.3f} m'.format(
                d[0], d[1], now.distance(d))

        print('Push now. Press Enter when the robot stands still at the end.')
        wait_enter(status)
        ticks, _ = enc.read()
        left, right = ticks[0] - start[0], ticks[1] - start[1]

        if not run_ok(enc, dropouts):
            continue
        if abs(left) + abs(right) < 200:
            print('    The wheels barely turned (left {:+d}, right {:+d} ticks).'.format(left, right))
            print('    Run discarded.')
            continue
        if left * right <= 0:
            print('    The wheels counted in different directions (left {:+d}, right {:+d}).'.format(left, right))
            print('    Pushed straight, both have to count the same way: one encoder')
            print('    counts backwards (ENC1_REVERSED / ENC2_REVERSED in the firmware)')
            print('    or doesn\'t count at all. Run discarded.')
            continue

        meters = ask_number('Distance on the tape in metres (e.g. 2.0): ', 20,
                            'More than 20 m? Type the distance in metres, e.g. 2.0')
        runs.append((left, right, meters))

        tpm = (abs(left) / meters, abs(right) / meters)
        print('    Run {}: left {:+d} ticks ({:.0f}/m), right {:+d} ticks ({:.0f}/m)'.format(
            len(runs), left, tpm[0], right, tpm[1]))
        print('    Left and right differ by {:.1f} %. Current odometry: {:.3f} m for {:.3f} m.'.format(
            abs(percent_off(tpm[0], tpm[1])), abs(now.distance((left, right))), meters))
        if left < 0:
            print('    Both counted backwards: fine if you pushed it backwards. If you pushed')
            print('    it forwards (towards the caster wheel), both encoders count backwards.')
        if max(tpm) / min(tpm) > 1.10:
            print('    That\'s a big difference for pushing straight. Did a wheel slide, or')
            print('    did the robot turn? If more runs show the same, one encoder misses')
            print('    ticks (loose wire, encoder VCC on 3.3 V instead of 5 V).')

        if not ask_yes('Another run? [y/N] '):
            return runs


def turn_test(enc, now):
    print()
    print(LINE)
    print('TEST 2 of 2: turning on the spot')
    print(LINE)
    print('Put the robot next to a straight line on the floor (tape, a tile seam)')
    print('and line one side of it up with the line. Start teleop in a second shell:')
    print('    rosrun teleop_twist_keyboard teleop_twist_keyboard.py')
    print('and turn the robot on the spot with j (left) or l (right) by 2 full turns')
    print('or more, until it lines up again. Press c in teleop a few times to turn')
    print('slower, then it\'s easier to stop on the line. Best: one run left, one right.')

    runs = []
    while True:
        print()
        print('Robot lined up? Press Enter.  (s + Enter = skip this test)')
        if wait_enter() == 's':
            return runs
        start, dropouts = enc.read()

        def status():
            ticks, _ = enc.read()
            d = (ticks[0] - start[0], ticks[1] - start[1])
            return 'left {:+6d}  right {:+6d} ticks   current odometry: {:+.0f} deg'.format(
                d[0], d[1], math.degrees(now.angle(d)))

        print('Turn now. Press Enter when it is lined up again and stands still.')
        wait_enter(status)
        ticks, _ = enc.read()
        left, right = ticks[0] - start[0], ticks[1] - start[1]

        if not run_ok(enc, dropouts):
            continue
        s_left, s_right = now.wheel_travel((left, right))
        if abs(s_right - s_left) < 0.3:
            print('    The robot barely turned (left {:+d}, right {:+d} ticks).'.format(left, right))
            print('    Run discarded.')
            continue

        turns = ask_number('Full turns the robot made (e.g. 2, or 1.5): ', 20,
                           'Type full turns, not degrees: 2 turns = 720 deg')
        runs.append((left, right, turns))

        angle = now.angle((left, right))
        print('    Run {}: {} turn(s) {}, current odometry: {:.0f} deg instead of {:.0f} deg'.format(
            len(runs), '{:g}'.format(turns),
            'left (counter-clockwise)' if angle > 0 else 'right (clockwise)',
            abs(math.degrees(angle)), 360 * turns))
        print('    If the robot actually turned the other way, the encoders are swapped.')
        centre = (s_left + s_right) / 2
        if abs(centre) > 0.2 * abs(s_right - s_left) / 2:
            print('    The robot didn\'t turn on the spot: the point between the wheels')
            print('    rolled {:.0f} cm. Still usable, but repeat it if you can.'.format(100 * abs(centre)))

        if not ask_yes('Another run? [y/N] '):
            return runs


# ---------------------------------------------------------------- summary

def summary(now, straight_runs, turn_runs):
    print()
    print(LINE)
    print('RESULT')
    print(LINE)

    if straight_runs:
        radius = radii_from_straight(straight_runs, now.ticks_per_rev)
    else:
        radius = now.radius
    measured = Settings(radius[0], radius[1], now.wheel_base, now.ticks_per_rev)
    if turn_runs:
        measured.wheel_base = wheel_base_from_turns(turn_runs, measured)

    def row(name, old, new, done, extra=''):
        if done:
            print('{:<14}{:7.1f} mm  ->  {:7.1f} mm  ({:+.1f} %){}'.format(
                name, 1000 * old, 1000 * new, percent_off(new, old), extra))
        else:
            print('{:<14}{:7.1f} mm  (not measured, unchanged)'.format(name, 1000 * old))

    print('              current          measured')
    for wheel, name in enumerate(('left radius', 'right radius')):
        tpm = 1 / measured.m_per_tick[wheel]
        row(name, now.radius[wheel], measured.radius[wheel], bool(straight_runs),
            '   {:.0f} ticks/m'.format(tpm))
    row('wheel base', now.wheel_base, measured.wheel_base, bool(turn_runs))

    print()
    for i, (left, right, meters) in enumerate(straight_runs, 1):
        print('straight {}: {:.3f} m, left {:+d}, right {:+d} ticks, current odometry {:.3f} m'.format(
            i, meters, left, right, abs(now.distance((left, right)))))
    for i, (left, right, turns) in enumerate(turn_runs, 1):
        print('turn {}:     {:g} turn(s), left {:+d}, right {:+d} ticks, current odometry {:.0f} deg'.format(
            i, turns, left, right, math.degrees(now.angle((left, right)))))

    if straight_runs:
        diff = abs(percent_off(measured.radius[0], measured.radius[1]))
        print()
        print('Left and right wheel differ by {:.1f} %.'.format(diff))
        if diff > 0.5:
            print('That is enough to make the robot drive a curve when both wheels turn')
            print('equally fast; with the measured radii in the firmware it drives straight.')
        off = max(abs(percent_off(r, n)) for r, n in zip(measured.radius, now.radius))
        if off > 10:
            print('The radius is {:.0f} % off the current value. If that doesn\'t fit the real'.format(off))
            print('wheels, the encoders don\'t give {:g} ticks per wheel turn: lift the'.format(now.ticks_per_rev))
            print('robot, turn a wheel exactly 10 times by hand and compare the count')
            print('with {:g}.'.format(10 * now.ticks_per_rev))
    if turn_runs:
        print()
        print('The wheel base is the effective value: if the wheels slip a bit while')
        print('turning, it comes out larger than the real wheel spacing. Use it anyway,')
        print('the odometry needs exactly this value.')

    print()
    print('navigation.launch (encoder_odometry_node):')
    print('    <param name="left_wheel_radius" value="{:.5f}"/>'.format(measured.radius[0]))
    print('    <param name="right_wheel_radius" value="{:.5f}"/>'.format(measured.radius[1]))
    print('    <param name="wheel_base" value="{:.4f}"/>'.format(measured.wheel_base))
    print('    <param name="ticks_per_revolution" value="{:g}"/>'.format(now.ticks_per_rev))
    print('firmware/umutbot_nano/umutbot_nano.ino (then flash the Nano again):')
    print('    const float WHEEL_RADIUS_LEFT = {:.5f};'.format(measured.radius[0]))
    print('    const float WHEEL_RADIUS_RIGHT = {:.5f};'.format(measured.radius[1]))
    print('    const float WHEEL_BASE = {:.4f};'.format(measured.wheel_base))
    return measured


def main():
    rospy.init_node('calibrate_odom', anonymous=True)
    now, found = current_settings()
    enc = Encoders()

    print('Waiting for encoder data ...')
    deadline = time.monotonic() + 5.0
    while not enc.ready():
        if time.monotonic() > deadline or rospy.is_shutdown():
            print('No data on /encoder_feedback_1 and /encoder_feedback_2.')
            print('Start the Nano bridge first: roslaunch umut_bot pico.launch')
            print('(or navigation.launch).')
            return 1
        time.sleep(0.1)

    print('Current odometry settings ({}):'.format(
        'from encoder_odometry_node' if found else 'defaults, navigation.launch is not running'))
    print('    wheel radius left {:.1f} mm, right {:.1f} mm, wheel base {:.1f} mm, {:g} ticks/turn'.format(
        1000 * now.radius[0], 1000 * now.radius[1], 1000 * now.wheel_base, now.ticks_per_rev))

    straight_runs = straight_test(enc, now)
    turn_runs = turn_test(enc, now)
    if not straight_runs and not turn_runs:
        print('Nothing measured.')
        return 0
    summary(now, straight_runs, turn_runs)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError, rospy.ROSInterruptException):
        print()
        print('Aborted.')
