import re
from dataclasses import dataclass
from typing import Optional

LINE_REGEX = re.compile(
    "^Valve ([A-Z]{2}) has flow rate=([0-9]+); tunnel(?:s)? lead(?:s)? to valve(?:s)? ((?:[A-Z]{2}(?:, )?)+)$"
)
MAX_TIME = 30


@dataclass
class Vertex:
    name: str
    flow_rate: int
    neighbors: list[str]


def parse_file(path: str) -> dict[str, Vertex]:
    results: dict[str, Vertex] = {}
    with open(path, "r") as f:
        for i, line in enumerate(f.readlines()):
            name, flow_str, neightbors_str = LINE_REGEX.match(line).groups()
            flow_rate = int(flow_str)
            neighbors = neightbors_str.split(", ")
            results[name] = Vertex(name=name, flow_rate=flow_rate, neighbors=neighbors)
    return results


@dataclass
class MapTracer:
    path_to_now: list[str]


def create_map(vertices: dict[str, Vertex]) -> dict[str, dict[str, MapTracer]]:
    """Creates a map to allow getting in the shortest path from every vertex
    to every vertex"""
    result = {}
    for vertex in vertices.values():
        map_tracers = {vertex.name: MapTracer(path_to_now=[])}
        active_tracer_positions = [vertex.name]
        while len(active_tracer_positions) > 0:
            new_active_tracer_positions = []
            for active_position in active_tracer_positions:
                neighbors = vertices[active_position].neighbors
                for neighbor in neighbors:
                    if neighbor in map_tracers:
                        # Already found path to there
                        continue
                    map_tracers[neighbor] = MapTracer(
                        path_to_now=map_tracers[active_position].path_to_now
                        + [neighbor]
                    )
                    new_active_tracer_positions.append(neighbor)
            active_tracer_positions = new_active_tracer_positions
        result[vertex.name] = map_tracers
    return result


@dataclass
class BfsPtr:
    time_remaining: int
    opened_valves: list[str]  # Sorted!
    pressure_released: int
    position: str
    path: list[tuple[str, int]]


Fingerprint = str


def ptr_fingerprint(ptr: BfsPtr) -> Fingerprint:
    return "".join(ptr.opened_valves) + "_" + str(ptr.position)


def is_better_than(p1: BfsPtr, p2: BfsPtr) -> bool:
    return (
        p1.pressure_released >= p2.pressure_released
        and p1.time_remaining >= p2.time_remaining
    )


BFS_DICT: dict[Fingerprint, [BfsPtr]] = {}
BEST_YET: int = 0
BEST_PTR: Optional[BfsPtr] = None


def get_bfs_kids(
    vertices: dict[str, Vertex],
    tunnels_map: dict[str, dict[str, MapTracer]],
    ptr: BfsPtr,
    non_zero_vertices: list[str],
):
    global BEST_YET
    global BEST_PTR
    children: list[BfsPtr] = []
    pressure_per_tick = sum([vertices[x].flow_rate for x in ptr.opened_valves])
    for vertex in non_zero_vertices:
        if vertex == ptr.position:
            continue
        dt = len(tunnels_map[ptr.position][vertex].path_to_now) + 1
        if vertex in ptr.opened_valves or ptr.time_remaining - dt < 0:
            possible_pressure = (
                ptr.pressure_released + ptr.time_remaining * pressure_per_tick
            )
            if possible_pressure > BEST_YET:
                BEST_YET = possible_pressure
                BEST_PTR = BfsPtr(
                    time_remaining=0,
                    opened_valves=ptr.opened_valves,
                    pressure_released=ptr.pressure_released
                    + ptr.time_remaining * pressure_per_tick,
                    position=ptr.position,
                    path=ptr.path
                )
            continue
        candidate_time_remaining = ptr.time_remaining - dt
        candidate_child = BfsPtr(
            time_remaining=candidate_time_remaining,
            opened_valves=sorted(ptr.opened_valves + [vertex]),
            pressure_released=dt * pressure_per_tick + ptr.pressure_released,
            position=vertex,
            path=ptr.path + [(vertex, candidate_time_remaining)]
        )
        fingerprint = ptr_fingerprint(candidate_child)
        if fingerprint not in BFS_DICT:
            BFS_DICT[fingerprint] = [candidate_child]
        else:
            child_handled = False
            for i, existing_ptr in list(enumerate(BFS_DICT[fingerprint])):
                if is_better_than(existing_ptr, candidate_child):
                    child_handled = True
                    break
                if is_better_than(candidate_child, existing_ptr):
                    BFS_DICT[fingerprint] = (
                        BFS_DICT[fingerprint][:i]
                        + BFS_DICT[fingerprint][i + 1 :]
                        + [candidate_child]
                    )
                    child_handled = True
                    break
            if not child_handled:
                BFS_DICT[fingerprint].append(candidate_child)


def solve() -> tuple[int, BfsPtr]:
    global BFS_DICT
    vertices = parse_file("input.txt")
    non_zero_vertices = [
        vertex.name for vertex in vertices.values() if vertex.flow_rate > 0
    ]
    tunnels_map = create_map(vertices)
    start_ptr = BfsPtr(
        time_remaining=MAX_TIME, opened_valves=[], pressure_released=0, position="AA", path=[("AA", MAX_TIME)]
    )
    BFS_DICT = {ptr_fingerprint(start_ptr): [start_ptr]}
    while len(BFS_DICT) > 0:
        copy_of_bfs_dict = {**BFS_DICT}
        BFS_DICT = {}
        for fingerprint, ptrs in copy_of_bfs_dict.items():
            if fingerprint == 'BBDDHHJJ_HH':
                a = 1
            for ptr in ptrs:
                get_bfs_kids(
                    vertices=vertices,
                    tunnels_map=tunnels_map,
                    ptr=ptr,
                    non_zero_vertices=non_zero_vertices,
                )
    return BEST_YET, BEST_PTR


if __name__ == "__main__":
    print(solve())
