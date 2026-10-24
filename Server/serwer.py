"""Control server for the line-following robots.

Clients set a goal node for a robot (/goal); the server plans paths for all
robots together with CBS (mapf.py) and hands the robots one command at a
time (/command), each confirmed by the robot (/done). A command is released
only when its dependencies are met, i.e. the robots that pass a node before
it in the plan have already left that node.

Run:  python serwer.py [--random]   (--random = random goals for testing)
"""
import json
import logging
import os
import random
import sys
import threading
import time

from flask import Flask, request

import mapf

# Silence Flask/Werkzeug HTTP logs
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Map of the board: (0,0) = top-left corner, x to the right, y down, in cm
# ---------------------------------------------------------------------------
NODES = {
    # intersections
    'T60': (60, 38), 'T100': (100, 38), 'T140': (140, 38),
    'B60': (60, 62), 'B100': (100, 62), 'B140': (140, 62),
    # inbound / outbound
    'IN1': (4, 38), 'IN2': (4, 62),
    'OUT1': (196, 38), 'OUT2': (196, 62),
    # chargers / racks
    'C1': (60, 4), 'C2': (100, 4), 'C3': (140, 4),
    'R1': (60, 96), 'R2': (100, 96), 'R3': (140, 96),
}

EDGES = [
    # horizontal lines
    ('IN1', 'T60'), ('T60', 'T100'), ('T100', 'T140'), ('T140', 'OUT1'),
    ('IN2', 'B60'), ('B60', 'B100'), ('B100', 'B140'), ('B140', 'OUT2'),
    # top-bottom connectors
    ('T60', 'B60'), ('T100', 'B100'), ('T140', 'B140'),
    # branches to the fields
    ('T60', 'C1'), ('T100', 'C2'), ('T140', 'C3'),
    ('B60', 'R1'), ('B100', 'R2'), ('B140', 'R3'),
]

# Fields = ends of lines (a robot stands on them facing the edge of the board)
FIELDS = {'IN1', 'IN2', 'OUT1', 'OUT2', 'C1', 'C2', 'C3', 'R1', 'R2', 'R3'}

# Ends of the short top-bottom connectors: two robots there at once could hit
# each other (when facing the same way or turning), so only one robot at a time
PAIRS = [('T60', 'B60'), ('T100', 'B100'), ('T140', 'B140')]

ROAD = mapf.RoadMap(NODES, EDGES, FIELDS, PAIRS)

# Side a robot turns around on a field: away from the neighbouring field, so it
# does not hit a robot standing there (IN1/IN2 and OUT1/OUT2 are side by side)
RETURN_SIDE = {'IN1': 'RIGHT', 'IN2': 'LEFT', 'OUT1': 'LEFT', 'OUT2': 'RIGHT'}

# ---------------------------------------------------------------------------
# Robot state
# ---------------------------------------------------------------------------
COMMAND_PAUSE = 2.0            # s between a confirmation and the next command
DEFAULT_ROBOT = '1'            # robot that does not send ?robot=

next_id = int(time.time())     # command ids keep growing across server restarts
state_lock = threading.Lock()
replan_needed = False          # last planning failed, retry after the next /done

# position, heading - where the robot is (after its last confirmed command)
# goal             - node the robot should get to (None = stay)
# actions          - planned actions not sent yet: {'seq', 'cmd', 'to', 'heading', 'deps'}
# current          - command sent to the robot, waiting for /done: {'id', 'cmd', 'actions'}
# done_seq         - seq of the last finished action; deps = [(robot, seq)] are
#                    met once that robot's done_seq reaches seq
robots = {
    rid: {'home': home, 'position': home, 'heading': ROAD.field_heading(home), 'goal': None,
          'actions': [], 'current': None, 'done_seq': 0, 'last_done_at': 0.0}
    for rid, home in (('1', 'C1'), ('3', 'C3'))   # robot id -> home charger
}

# State saved after every change. After a server restart the robot gets the
# same command in progress (same id), confirms it with /done and the position
# is right again.
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'server_state.json')
SAVED_KEYS = ('position', 'heading', 'goal', 'actions', 'current', 'done_seq')


def save_state():
    """Call with state_lock held."""
    data = {'next_id': next_id,
            'robots': {rid: {k: r[k] for k in SAVED_KEYS} for rid, r in robots.items()}}
    tmp = STATE_FILE + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, STATE_FILE)   # atomic: never leaves a half-written file


def load_state():
    global next_id
    if not os.path.exists(STATE_FILE):
        return
    with open(STATE_FILE) as f:
        data = json.load(f)
    if any('heading' not in saved for saved in data['robots'].values()):
        sys.exit(f"{STATE_FILE} is from the old single-robot server. Put the robots "
                 f"on their chargers (C1, C2, C3) and delete the file.")
    next_id = max(next_id, data['next_id'])
    for rid, saved in data['robots'].items():
        if rid in robots:
            robots[rid].update(saved)
            # JSON turns tuples into lists
            for a in robots[rid]['actions'] + (robots[rid]['current'] or {}).get('actions', []):
                a['deps'] = [tuple(d) for d in a['deps']]
            print(f"[STATE] robot {rid} at {saved['position']} facing {saved['heading']}, "
                  f"goal {saved['goal']}, in progress: {(saved['current'] or {}).get('cmd')}")


# ---------------------------------------------------------------------------
# Planning
# ---------------------------------------------------------------------------
def end_state(r):
    """(node, heading) the robot will be in once its current command is done."""
    if r['current']:
        last = r['current']['actions'][-1]
        return last['to'], last['heading']
    return r['position'], r['heading']


def replan():
    """Plan all robots from their end states to their goals. Call with state_lock held.

    Actions already sent stay as they are; everything not sent yet is replaced.
    """
    global replan_needed
    ids = list(robots)
    starts = [end_state(robots[rid]) for rid in ids]
    goals = [robots[rid]['goal'] or start[0] for rid, start in zip(ids, starts)]

    t0 = time.perf_counter()
    paths = mapf.cbs(ROAD, starts, goals)
    ms = (time.perf_counter() - t0) * 1000
    if paths is None:
        replan_needed = True
        print(f"[PLAN] no solution from {dict(zip(ids, starts))} to {dict(zip(ids, goals))} "
              f"({ms:.0f} ms), retry later")
        return False
    replan_needed = False

    # seq numbers continue after the command in progress
    base = {rid: (robots[rid]['current']['actions'][-1]['seq'] if robots[rid]['current']
                  else robots[rid]['done_seq']) for rid in ids}

    # nodes a robot is driving through right now (and their partners)
    # are free only once it confirms
    in_flight = {}
    for rid in ids:
        cur = robots[rid]['current']
        if cur:
            for node in [robots[rid]['position']] + [a['to'] for a in cur['actions'][:-1]]:
                in_flight[node] = (rid, base[rid])
                if node in ROAD.partner:
                    in_flight[ROAD.partner[node]] = (rid, base[rid])

    for i, (rid, actions) in enumerate(zip(ids, mapf.build_adg(ROAD, paths))):
        for k, a in enumerate(actions):
            a['seq'] = base[rid] + k + 1
            a['deps'] = [(ids[j], base[ids[j]] + idx + 1) for j, idx in a['deps']]
            if a['cmd'] == 'FORWARD' and a['to'] in in_flight and in_flight[a['to']][0] != rid:
                a['deps'].append(in_flight[a['to']])
        robots[rid]['actions'] = actions
    print(f"[PLAN] CBS {ms:.1f} ms: " + '; '.join(
        f"{rid}: {', '.join(a['cmd'] + '->' + a['to'] for a in robots[rid]['actions']) or '-'}"
        for rid in ids))
    return True


def deps_met(action):
    return all(robots[rid]['done_seq'] >= seq for rid, seq in action['deps'])


def next_command(r):
    """Actions that can be sent now as one robot command, or [].

    Straight drives through intersections are merged (FORWARD_AFTER_LINE 2),
    as long as nothing has to be waited for at the intersections in between.
    """
    actions = r['actions']
    if not actions or not deps_met(actions[0]):
        return []
    batch = [actions[0]]
    if batch[0]['cmd'] == 'FORWARD':
        for a in actions[1:]:
            if a['cmd'] != 'FORWARD' or batch[-1]['to'] in FIELDS or not deps_met(a):
                break
            batch.append(a)
    return batch


def command_text(batch):
    cmd = batch[0]['cmd']
    if cmd == 'FORWARD':
        # onto a field: stop on the stop bar; to an intersection: stop with the axle on it
        cmd = 'FORWARD' if batch[-1]['to'] in FIELDS else 'FORWARD_AFTER_LINE'
        if len(batch) > 1:
            cmd += f' {len(batch)}'
    elif cmd == 'RETURN':
        cmd += '_' + RETURN_SIDE.get(batch[0]['to'], 'LEFT')
    return cmd


def set_goal(robot, goal):
    """Returns an error message or None."""
    if robot not in robots:
        return f"Unknown robot: {robot}"
    if goal not in NODES:
        return f"Unknown node: {goal}"
    blocked = {goal, ROAD.partner.get(goal, goal)}
    with state_lock:
        for rid, r in robots.items():
            # two robots can never end on the same node (or on a pair)
            staying = r['goal'] in (None, r['position'])
            if rid != robot and (r['goal'] in blocked or (staying and r['position'] in blocked)):
                return f"{goal} is taken by robot {rid}"
        robots[robot]['goal'] = goal
        print(f"[GOAL] robot {robot} -> {goal}")
        replan()
        save_state()
    return None


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------
@app.route('/command', methods=['GET'])
def get_command():
    """The ESP32 polls its current command as "<id> <COMMAND>".

    Until it confirms it with /done it keeps getting the same one (the robot
    recognises the id it has already executed). Nothing to do = "0 STOP".
    """
    global next_id
    r = robots.get(request.args.get('robot', DEFAULT_ROBOT))
    if r is None:
        return "0 STOP", 200
    with state_lock:
        paused = time.monotonic() - r['last_done_at'] < COMMAND_PAUSE
        if r['current'] is None and not paused:
            batch = next_command(r)
            if batch:
                r['current'] = {'id': next_id, 'cmd': command_text(batch), 'actions': batch}
                r['actions'] = r['actions'][len(batch):]
                next_id += 1
                save_state()
                print(f"[SENT] robot {request.args.get('robot', DEFAULT_ROBOT)} "
                      f"#{r['current']['id']} {r['current']['cmd']} -> {batch[-1]['to']}")
        if r['current'] is not None:
            return f"{r['current']['id']} {r['current']['cmd']}", 200
    return "0 STOP", 200


@app.route('/done', methods=['GET', 'POST'])
def command_done():
    """The robot confirms a command: /done?id=<id>."""
    rid = request.args.get('robot', DEFAULT_ROBOT)
    r = robots.get(rid)
    cmd_id = request.args.get('id', type=int)
    with state_lock:
        if r is not None and r['current'] is not None and r['current']['id'] == cmd_id:
            last = r['current']['actions'][-1]
            r['position'], r['heading'], r['done_seq'] = last['to'], last['heading'], last['seq']
            print(f"[DONE] robot {rid} #{cmd_id} {r['current']['cmd']} "
                  f"(at {r['position']} facing {r['heading']})")
            r['current'] = None
            r['last_done_at'] = time.monotonic()
            if replan_needed:
                replan()
            save_state()
    return "OK", 200


@app.route('/goal', methods=['GET', 'POST'])
def goal():
    """E.g. /goal?robot=2&to=R2 - send robot 2 to R2 (replans all robots)."""
    error = set_goal(request.args.get('robot', DEFAULT_ROBOT), request.args.get('to', '').upper())
    if error:
        return {'error': error}, 400
    return {'ok': True}, 200


@app.route('/status', methods=['GET'])
def status():
    """Robot states, e.g. for the warehouse: {"1": {"position": "C1", "busy": false, ...}}."""
    with state_lock:
        return {rid: {'position': r['position'], 'heading': r['heading'], 'home': r['home'],
                      'goal': r['goal'],
                      'busy': bool(r['current'] or r['actions'])
                              or (r['goal'] is not None and r['goal'] != r['position'])}
                for rid, r in robots.items()}, 200


def terminal_input_loop():
    print("\n--- CONTROL SERVER RUNNING ---")
    print("Type '<robot> <goal>' (e.g. 2 R2).\n")

    while True:
        try:
            parts = input(">>> ").strip().upper().split()
        except EOFError:
            break
        if len(parts) != 2:
            print("[ERROR] expected '<robot> <goal>'")
            continue
        error = set_goal(*parts)
        if error:
            print(f"[ERROR] {error}")


def random_routes_loop():
    """Forever: when a robot reaches its goal, send it to a random free field."""
    while True:
        for rid, r in robots.items():
            with state_lock:
                idle = not r['current'] and not r['actions'] and r['goal'] in (None, r['position'])
                taken = {o['goal'] for o in robots.values()} | {o['position'] for o in robots.values()}
            if idle:
                set_goal(rid, random.choice(sorted(FIELDS - taken)))
        time.sleep(0.5)


if __name__ == '__main__':
    load_state()
    with state_lock:
        # the saved plan may come from older rules - plan the rest again
        if any(r['actions'] for r in robots.values()):
            replan()
            save_state()

    # By default goals come from the warehouse (warehouse.py); --random = random goals
    if '--random' in sys.argv:
        t = threading.Thread(target=random_routes_loop, daemon=True)
        t.start()

    app.run(host='0.0.0.0', port=8080, debug=False)
