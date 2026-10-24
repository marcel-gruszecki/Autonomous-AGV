"""Multi-agent path finding for line-following robots.

The map is a graph of nodes (intersections and end-of-line fields) with
coordinates, so every edge has a compass direction. A robot state is
(node, heading) and in every time step a robot performs one action:

    FORWARD     drive along the line to the neighbouring node ahead
    TURN_LEFT   rotate 90 degrees on an intersection
    TURN_RIGHT
    RETURN      rotate 180 degrees on a field (end of a line)
    WAIT        stay

Planning: Conflict-Based Search (Sharon et al., 2015) with space-time A* as
the low level. Conflicts are 1-robust: a robot may not enter a node in the
step right after another robot was there, so robots never follow each other
nose to tail and the plan never contains cycles of robots that move at once.
Paired nodes (the two ends of a short connector) count as one place: robots
standing on both ends at once could hit each other, so they never do.

Execution: real robots do not move in lock-step, so the timed plan is turned
into an Action Dependency Graph (Hoenig et al., 2019). The plan only fixes
the ORDER in which robots pass each node; a robot may enter a node once the
robot that was there before it (in the plan) has left. Delays of one robot
then only make the others wait instead of causing collisions.
"""
import heapq
from collections import deque
from itertools import count

HEADINGS = 'NESW'   # clockwise; y grows downwards (screen coordinates)
VECTORS = {'N': (0, -1), 'E': (1, 0), 'S': (0, 1), 'W': (-1, 0)}


def turn(heading, quarter_turns):
    """quarter_turns: +1 = right, -1 = left, 2 = back."""
    return HEADINGS[(HEADINGS.index(heading) + quarter_turns) % 4]


class RoadMap:
    """Graph of the line track with the robot actions defined on it."""

    def __init__(self, nodes, edges, fields, pairs=()):
        """nodes: {name: (x, y)}, edges: [(a, b)], fields: dead-end nodes,
        pairs: [(a, b)] nodes too close to be occupied by two robots at once."""
        self.nodes = nodes
        self.fields = set(fields)
        self.partner = {}   # node -> the other node of its pair
        for a, b in pairs:
            self.partner[a], self.partner[b] = b, a
        self.ahead = {}   # (node, heading) -> neighbouring node in that direction
        for a, b in edges:
            self.ahead[(a, self.direction(a, b))] = b
            self.ahead[(b, self.direction(b, a))] = a
        self._heuristics = {}

    def direction(self, a, b):
        """Compass heading of the straight edge a -> b."""
        (ax, ay), (bx, by) = self.nodes[a], self.nodes[b]
        sign = lambda v: (v > 0) - (v < 0)
        vector = (sign(bx - ax), sign(by - ay))
        return next(h for h, v in VECTORS.items() if v == vector)

    def field_heading(self, field):
        """A robot that drove onto a field faces away from the track."""
        neighbour = next(b for (a, _), b in self.ahead.items() if a == field)
        return self.direction(neighbour, field)

    def successors(self, state):
        """All (action, next_state) pairs available in state."""
        node, heading = state
        result = [('WAIT', state)]
        if (node, heading) in self.ahead:
            result.append(('FORWARD', (self.ahead[(node, heading)], heading)))
        if node in self.fields:
            result.append(('RETURN', (node, turn(heading, 2))))
        else:
            # turn only towards a line, the robot stops turning when it sees one
            for action, quarter in (('TURN_LEFT', -1), ('TURN_RIGHT', 1)):
                new_heading = turn(heading, quarter)
                if (node, new_heading) in self.ahead:
                    result.append((action, (node, new_heading)))
        return result

    def heuristic(self, goal):
        """{state: number of actions to reach goal}, ignoring other robots.

        Backward BFS over the state graph; computed once per goal node.
        """
        if goal not in self._heuristics:
            predecessors = {}
            for node in self.nodes:
                for heading in HEADINGS:
                    for _, nxt in self.successors((node, heading)):
                        predecessors.setdefault(nxt, []).append((node, heading))
            dist = {(goal, h): 0 for h in HEADINGS}
            queue = deque(dist)
            while queue:
                state = queue.popleft()
                for prev in predecessors.get(state, []):
                    if prev not in dist:
                        dist[prev] = dist[state] + 1
                        queue.append(prev)
            self._heuristics[goal] = dist
        return self._heuristics[goal]


# ---------------------------------------------------------------------------
# Low level: space-time A* for one robot
# ---------------------------------------------------------------------------
def space_time_astar(road, start, goal, constraints):
    """Shortest list of states (one per time step) from start to goal node.

    constraints: set of (node, t) this robot must not occupy.
    The path ends at the goal and the robot stays there afterwards, so it may
    only finish after the last constraint on the goal node.
    """
    h = road.heuristic(goal)
    if start not in h:
        return None
    goal_free_from = 1 + max((t for node, t in constraints if node == goal), default=-1)
    max_t = goal_free_from + 4 * len(road.nodes) * len(HEADINGS)   # search horizon

    tie = count()
    open_list = [(h[start], next(tie), start, 0)]
    parent = {(start, 0): None}
    while open_list:
        _, _, state, t = heapq.heappop(open_list)
        if state[0] == goal and t >= goal_free_from:
            path = [state]
            key = (state, t)
            while parent[key] is not None:
                key = parent[key]
                path.append(key[0])
            return path[::-1]
        if t >= max_t:
            continue
        for _, nxt in road.successors(state):
            key = (nxt, t + 1)
            if key in parent or (nxt[0], t + 1) in constraints or nxt not in h:
                continue
            parent[key] = (state, t)
            heapq.heappush(open_list, (t + 1 + h[nxt], next(tie), nxt, t + 1))
    return None


# ---------------------------------------------------------------------------
# High level: Conflict-Based Search
# ---------------------------------------------------------------------------
def _node_at(path, t):
    return path[min(t, len(path) - 1)][0]


def _start_stay(path):
    """Number of steps the robot stays on its start node before leaving it."""
    return next((t for t, (node, _) in enumerate(path) if node != path[0][0]), float('inf'))


def find_conflict(road, paths):
    """First conflict as ((agent, node, t), (agent, node, t)) or None.

    Vertex conflict: two robots on the same node (or on the two nodes of a
    pair) at the same time.
    Following conflict: a robot enters a node (or its partner) one step after
    another robot was there (this also covers two robots swapping places on
    an edge).
    Robots that already stand on both nodes of a pair at the start cannot
    help it, that is not a conflict until one of them drives away.
    """
    horizon = max(len(p) for p in paths)
    stay = [_start_stay(p) for p in paths]
    for t in range(horizon):
        for i in range(len(paths)):
            for j in range(i + 1, len(paths)):
                for a, ta, b, tb in ((i, t, j, t), (i, t, j, t + 1), (j, t, i, t + 1)):
                    na, nb = _node_at(paths[a], ta), _node_at(paths[b], tb)
                    if na == nb or (road.partner.get(na) == nb
                                    and not (ta < stay[a] and tb < stay[b])):
                        return (a, na, ta), (b, nb, tb)
    return None


def cbs(road, starts, goals, max_expansions=2000):
    """Optimal (sum of costs) collision-free paths, one list of states per robot.

    starts: [(node, heading)], goals: [node]. Returns None if no solution
    was found within max_expansions high-level nodes.
    """
    constraints = [set() for _ in starts]
    paths = [space_time_astar(road, s, g, c) for s, g, c in zip(starts, goals, constraints)]
    if any(p is None for p in paths):
        return None

    tie = count()
    cost = lambda ps: sum(len(p) - 1 for p in ps)
    open_list = [(cost(paths), next(tie), constraints, paths)]
    for _ in range(max_expansions):
        if not open_list:
            return None
        _, _, constraints, paths = heapq.heappop(open_list)
        conflict = find_conflict(road, paths)
        if conflict is None:
            return paths
        # branch: forbid the conflicting position to one robot or to the other
        for agent, node, t in conflict:
            child = [set(c) for c in constraints]
            child[agent].add((node, t))
            new_path = space_time_astar(road, starts[agent], goals[agent], child[agent])
            if new_path is None:
                continue
            child_paths = list(paths)
            child_paths[agent] = new_path
            heapq.heappush(open_list, (cost(child_paths), next(tie), child, child_paths))
    return None


# ---------------------------------------------------------------------------
# Plan -> actions with dependencies (Action Dependency Graph)
# ---------------------------------------------------------------------------
def path_to_actions(path):
    """Timed path -> [(t, action, state_after)], waits are dropped."""
    actions = []
    for t in range(len(path) - 1):
        (node, heading), (nxt, nxt_heading) = path[t], path[t + 1]
        if nxt != node:
            action = 'FORWARD'
        elif nxt_heading == heading:
            continue   # WAIT: the dependencies take care of timing
        elif nxt_heading == turn(heading, 2):
            action = 'RETURN'
        else:
            action = 'TURN_RIGHT' if nxt_heading == turn(heading, 1) else 'TURN_LEFT'
        actions.append((t, action, path[t + 1]))
    return actions


def build_adg(road, paths):
    """For every robot a list of actions {'cmd', 'to', 'heading', 'deps'}.

    deps = [(agent, action_index)]: actions of other robots that must be
    finished first. A robot entering a node waits for every robot that was on
    that node (or on its partner) earlier in the plan to drive away from it.
    """
    timed = [path_to_actions(p) for p in paths]
    result = []
    for j, actions in enumerate(timed):
        result.append([])
        for t, action, (node, heading) in actions:
            deps = []
            if action == 'FORWARD':
                arrive = t + 1
                for i, other in enumerate(paths):
                    if i == j:
                        continue
                    blocking = {node, road.partner.get(node, node)}
                    visits = [s for s in range(arrive) if _node_at(other, s) in blocking]
                    if not visits:
                        continue
                    # the move of robot i away from the node after its last visit
                    # (1-robust plans guarantee it ends before robot j arrives)
                    leave = next(k for k, (s, a, st) in enumerate(timed[i])
                                 if a == 'FORWARD' and s >= visits[-1])
                    deps.append((i, leave))
            result[j].append({'cmd': action, 'to': node, 'heading': heading, 'deps': deps})
    return result
