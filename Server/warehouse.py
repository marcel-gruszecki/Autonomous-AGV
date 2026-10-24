"""Warehouse layer on top of the robot server (serwer.py).

Receives events "goods arrived at IN" and "goods requested at OUT", keeps
the stock of racks R1-R3 (one item type per rack) and turns them into robot
tasks for the robots. The server (/goal) plans collision-free paths for all
robots, the warehouse only decides which robot goes where.

Run (server must be running):
    python warehouse.py                     # terminal only
    python warehouse.py --simulate          # + random events (continuous work)
    python warehouse.py --replay logs/session_20261024_150000.json

Every run (except replay) records its events to logs/session_<time>.json,
so the same work can be replayed later with --replay.
"""
import argparse
import json
import os
import random
import threading
import time
from datetime import datetime
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import urlopen

SERVER_URL = 'http://127.0.0.1:8080'

ITEMS = {'A': 'R1', 'B': 'R2', 'C': 'R3'}   # item type -> its rack
RACK_CAPACITY = 5
IN_STATIONS = ('IN1', 'IN2')
OUT_STATIONS = ('OUT1', 'OUT2')

POLL_S = 1.0                    # how often the robot status is checked
SIM_INTERVAL = (5.0, 15.0)     # simulation: seconds between random events

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(BASE_DIR, 'warehouse_state.json')
LOG_DIR = os.path.join(BASE_DIR, 'logs')


def log(msg):
    print(f"{datetime.now():%H:%M:%S} {msg}", flush=True)


def save_json(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)   # atomic: never leaves a half-written file


# ---------------------------------------------------------------------------
# Server communication
# ---------------------------------------------------------------------------
_server_down = False   # report an outage once, not on every poll


def server_get(path, **params):
    """GET to the robot server, returns parsed JSON or None if it is unreachable."""
    global _server_down
    url = SERVER_URL + path + ('?' + urlencode(params) if params else '')
    try:
        with urlopen(url, timeout=3) as resp:
            data = json.loads(resp.read())
    except HTTPError as e:   # the server answered, but refused the request
        log(f"[SERVER] {path} {params} refused: {e.read().decode(errors='replace')}")
        return None
    except (URLError, OSError, ValueError) as e:
        if not _server_down:
            log(f"[SERVER] {path} failed: {e} (retrying)")
        _server_down = True
        return None
    if _server_down:
        log("[SERVER] connection restored")
    _server_down = False
    return data


# ---------------------------------------------------------------------------
# Warehouse state and task logic
# ---------------------------------------------------------------------------
def empty_state():
    return {
        'racks': {item: 0 for item in ITEMS},   # pieces on each rack
        'inbound': [],    # goods waiting at IN: {"item", "station"}, FIFO
        'orders': [],     # requests waiting at OUT: {"item", "station"}, FIFO
        'tasks': {},      # robot id -> task in progress, see choose_task()
    }


def migrate(state):
    """Files from the single-robot version kept one 'task' (robot 1)."""
    if 'task' in state:
        task = state.pop('task')
        state['tasks'] = {'1': task} if task else {}
    return state


class Warehouse:
    def __init__(self, recorder=None):
        self.lock = threading.RLock()
        self.recorder = recorder
        self.state = empty_state()
        if os.path.exists(STATE_FILE):
            with open(STATE_FILE) as f:
                self.state = migrate(json.load(f))
            log(f"[STATE] loaded from {os.path.basename(STATE_FILE)}")

    def save(self):
        save_json(STATE_FILE, self.state)

    # --- events -----------------------------------------------------------
    def add_event(self, kind, item, station):
        """kind = 'delivery' (goods arrived at IN) or 'order' (goods requested at OUT)."""
        item, station = item.upper(), station.upper()
        stations = IN_STATIONS if kind == 'delivery' else OUT_STATIONS
        if item not in ITEMS:
            raise ValueError(f"Unknown item {item}, known: {', '.join(ITEMS)}")
        if station not in stations:
            raise ValueError(f"Wrong station {station}, expected one of: {', '.join(stations)}")

        with self.lock:
            target = self.state['inbound'] if kind == 'delivery' else self.state['orders']
            target.append({'item': item, 'station': station})
            self.save()
        if self.recorder:
            self.recorder.record(kind, item, station)
        log(f"[EVENT] {kind} {item} at {station}")

    # --- tasks ------------------------------------------------------------
    def target(self, rid, robot):
        """Field the robot is heading to: its task's pickup/drop or its charger."""
        t = self.state['tasks'].get(rid)
        if t is None:
            return robot['home']
        return t['pickup'] if t['stage'] == 'pickup' else t['drop']

    def choose_task(self, blocked):
        """Pick the next task and remove its event from the waiting lists.

        Orders go first: from the rack if it has stock, otherwise cross-docking
        straight from IN to OUT if the item is waiting there. Then waiting goods
        are stored on racks that are not full. Tasks of other robots count as
        already done (stock reserved), pickups on blocked fields are skipped.
        """
        s = self.state
        incoming = {item: 0 for item in ITEMS}   # being carried to the rack
        outgoing = {item: 0 for item in ITEMS}   # will be taken from the rack
        for t in s['tasks'].values():
            if t and t['kind'] == 'store':
                incoming[t['item']] += 1
            if t and t['kind'] == 'deliver' and t['stage'] == 'pickup':
                outgoing[t['item']] += 1

        for order in s['orders']:
            item = order['item']
            if s['racks'][item] - outgoing[item] > 0 and ITEMS[item] not in blocked:
                s['orders'].remove(order)
                return {'kind': 'deliver', 'item': item,
                        'pickup': ITEMS[item], 'drop': order['station'], 'stage': 'pickup'}
            if s['racks'][item] - outgoing[item] > 0:
                continue   # stock is there, the rack is just busy
            for inb in s['inbound']:
                if inb['item'] == item and inb['station'] not in blocked:
                    s['orders'].remove(order)
                    s['inbound'].remove(inb)
                    return {'kind': 'crossdock', 'item': item,
                            'pickup': inb['station'], 'drop': order['station'], 'stage': 'pickup'}

        for inb in s['inbound']:
            item = inb['item']
            if s['racks'][item] + incoming[item] < RACK_CAPACITY and inb['station'] not in blocked:
                s['inbound'].remove(inb)
                return {'kind': 'store', 'item': item,
                        'pickup': inb['station'], 'drop': ITEMS[item], 'stage': 'pickup'}
        return None

    def step(self, rid, robot, others):
        """One control step for robot rid; robot/others = entries of the server's /status.

        Progress is derived from the robot position, so after a restart of the
        server or the warehouse the task just continues from where it is.
        """
        if robot['busy']:
            return
        with self.lock:
            s = self.state
            pos = robot['position']
            # fields other robots stand on or were already sent to; two robots
            # never share one, whoever asks first goes first
            blocked = {o['position'] for o in others.values()}
            blocked |= {o['goal'] for o in others.values() if o['goal']}

            if s['tasks'].get(rid) is None:
                s['tasks'][rid] = self.choose_task(blocked)
                if s['tasks'][rid] is None:
                    if pos != robot['home']:
                        log(f"[IDLE] robot {rid}: nothing to do, going home to {robot['home']}")
                        server_get('/goal', robot=rid, to=robot['home'])
                    return
                t = s['tasks'][rid]
                log(f"[TASK] robot {rid}: {t['kind']} {t['item']}: {t['pickup']} -> {t['drop']}")
                self.save()

            t = s['tasks'][rid]
            if t['stage'] == 'pickup' and pos == t['pickup']:
                if t['kind'] == 'deliver':
                    s['racks'][t['item']] -= 1
                t['stage'] = 'drop'
                log(f"[PICKUP] robot {rid}: {t['item']} at {pos}")
                self.save()

            if t['stage'] == 'drop' and pos == t['drop']:
                if t['kind'] == 'store':
                    s['racks'][t['item']] += 1
                log(f"[DROP] robot {rid}: {t['item']} at {pos} ({t['kind']} done)")
                s['tasks'][rid] = None
                self.save()
                return

            goal = self.target(rid, robot)
            if goal not in blocked:   # otherwise wait until the field is free
                server_get('/goal', robot=rid, to=goal)

    def run(self):
        """Control loop: polls the server and drives the robots."""
        while True:
            status = server_get('/status')
            for rid, robot in (status or {}).items():
                self.step(rid, robot, {k: v for k, v in status.items() if k != rid})
            time.sleep(POLL_S)

    def print_status(self):
        with self.lock:
            s = self.state
            racks = ', '.join(f"{ITEMS[i]}({i}): {n}/{RACK_CAPACITY}" for i, n in s['racks'].items())
            print(f"  racks:   {racks}")
            print("  inbound:", ', '.join(e['item'] + '@' + e['station'] for e in s['inbound']))
            print("  orders: ", ', '.join(e['item'] + '@' + e['station'] for e in s['orders']))
            for rid, t in sorted(s['tasks'].items()):
                print(f"  robot {rid}: {t}")


# ---------------------------------------------------------------------------
# Recording, replay and simulation
# ---------------------------------------------------------------------------
class Recorder:
    """Writes the session events with times relative to the session start."""

    def __init__(self, initial_state):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = os.path.join(LOG_DIR, f"session_{datetime.now():%Y%m%d_%H%M%S}.json")
        self.start = time.monotonic()
        self.data = {'initial_state': initial_state, 'events': []}
        save_json(self.path, self.data)
        log(f"[RECORD] {os.path.relpath(self.path, BASE_DIR)}")

    def record(self, kind, item, station):
        self.data['events'].append({'t': round(time.monotonic() - self.start, 1),
                                    'type': kind, 'item': item, 'station': station})
        save_json(self.path, self.data)


def replay_loop(wh, events, speed):
    start = time.monotonic()
    for ev in events:
        delay = ev['t'] / speed - (time.monotonic() - start)
        if delay > 0:
            time.sleep(delay)
        wh.add_event(ev['type'], ev['item'], ev['station'])
    log("[REPLAY] all events sent")


def simulate_loop(wh, rng):
    while True:
        time.sleep(rng.uniform(*SIM_INTERVAL))
        item = rng.choice(sorted(ITEMS))
        if rng.random() < 0.5:
            wh.add_event('delivery', item, rng.choice(IN_STATIONS))
        else:
            wh.add_event('order', item, rng.choice(OUT_STATIONS))


# ---------------------------------------------------------------------------
# Terminal
# ---------------------------------------------------------------------------
HELP = """Commands:
  in <item> [IN1|IN2]      goods arrived (default IN1), e.g. "in A IN2"
  out <item> [OUT1|OUT2]   goods requested (default OUT1), e.g. "out B"
  status                   show the warehouse state
  quit                     exit (state is kept in warehouse_state.json)
Items: """ + ', '.join(f"{i} -> {r}" for i, r in ITEMS.items())


def terminal_loop(wh):
    print(HELP)
    while True:
        try:
            parts = input(">>> ").split()
        except (EOFError, KeyboardInterrupt):
            break
        if not parts:
            continue
        cmd = parts[0].lower()
        try:
            if cmd == 'in' and len(parts) in (2, 3):
                wh.add_event('delivery', parts[1], parts[2] if len(parts) == 3 else 'IN1')
            elif cmd == 'out' and len(parts) in (2, 3):
                wh.add_event('order', parts[1], parts[2] if len(parts) == 3 else 'OUT1')
            elif cmd == 'status':
                wh.print_status()
            elif cmd in ('quit', 'exit'):
                break
            else:
                print(HELP)
        except ValueError as e:
            print(f"[ERROR] {e}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--simulate', action='store_true', help='generate random events')
    mode.add_argument('--replay', metavar='FILE', help='replay events from a recorded session')
    parser.add_argument('--seed', type=int, help='random seed for --simulate')
    parser.add_argument('--speed', type=float, default=1.0, help='time factor for --replay')
    args = parser.parse_args()

    if args.replay:
        with open(args.replay) as f:
            session = json.load(f)
        wh = Warehouse()
        if any(wh.state['tasks'].values()):
            parser.error("tasks are still in progress; finish them first (run without --replay)")
        # start from the same stock as the recorded session
        wh.state = migrate(session['initial_state'])
        wh.state['tasks'] = {}
        wh.save()
        threading.Thread(target=replay_loop, args=(wh, session['events'], args.speed), daemon=True).start()
    else:
        wh = Warehouse()
        with wh.lock:
            wh.recorder = Recorder(json.loads(json.dumps(wh.state)))
        if args.simulate:
            threading.Thread(target=simulate_loop, args=(wh, random.Random(args.seed)), daemon=True).start()

    threading.Thread(target=wh.run, daemon=True).start()
    terminal_loop(wh)


if __name__ == '__main__':
    main()
