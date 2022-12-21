import random
import re
import time
from dataclasses import dataclass
from typing import Optional, TypeVar, cast
from tqdm import tqdm

LINE_REGEX = re.compile(
    "^Valve ([A-Z]{2}) has flow rate=([0-9]+); tunnel(?:s)? lead(?:s)? to valve(?:s)? ((?:[A-Z]{2}(?:, )?)+)$"
)
MAX_TIME = 26


@dataclass
class Vertex:
    name: str
    flow_rate: int
    neighbors: list[str]


def parse_file(path: str) -> dict[str, Vertex]:
    results: dict[str, Vertex] = {}
    with open(path, "r") as f:
        for i, line in enumerate(f.readlines()):
            match = LINE_REGEX.match(line)
            if match is None:
                raise RuntimeError("BAD")
            name, flow_str, neightbors_str = match.groups()
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
    path: list[tuple[str, int]]
    p1_dest: str
    p1_time_to_dest: int
    p2_dest: str
    p2_time_to_dest: int


Fingerprint = str


def ptr_fingerprint(ptr: BfsPtr) -> Fingerprint:
    return "".join(ptr.opened_valves) + "_" + ptr.p1_dest + ptr.p2_dest


def is_better_than(p1: BfsPtr, p2: BfsPtr) -> bool:
    return (
        p1.pressure_released >= p2.pressure_released
        and p1.time_remaining >= p2.time_remaining
        and p1.p1_time_to_dest
        <= p2.p1_time_to_dest + p1.time_remaining - p2.time_remaining
        and p1.p2_time_to_dest
        <= p2.p2_time_to_dest + p1.time_remaining - p2.time_remaining
    )


BFS_DICT: dict[Fingerprint, list[BfsPtr]] = {}
BEST_YET: int = 0
BEST_PTR: Optional[BfsPtr] = None

TIME_EXPONENT = 2  # Pressure released is proportional to time squared up to diminishing returns on saturating the map
BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH: dict[int, float] = {}
PRESSURE_TO_TIME_MIN_PATH_LENGTH = 2
PRESSURE_TO_TIME_SQUARED_ALLOWED_RATIO_TO_BEST_BY_PATH_LENGTH = {
    # Kinda guess, just a little bit of fine tune
    2: 0.5,
    3: 0.55,
    4: 0.6,
    5: 0.65,
    6: 0.7,
    7: 0.75,
    8: 0.8,
    9: 0.85,
    10: 0.86,
    11: 0.88,
    12: 0.89,
    13: 0.9,
    14: 0.95,
}


def get_bfs_kids(
    vertices: dict[str, Vertex],
    tunnels_map: dict[str, dict[str, MapTracer]],
    ptr: BfsPtr,
    non_zero_vertices: list[str],
) -> None:
    global BEST_YET
    global BEST_PTR
    global BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH
    ptr_path_len = len(ptr.path)
    if (
        ptr_path_len >= PRESSURE_TO_TIME_MIN_PATH_LENGTH
        and ptr_path_len in BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH
        and (ptr.pressure_released / (MAX_TIME - ptr.time_remaining) ** TIME_EXPONENT)
        / BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH[ptr_path_len]
        < PRESSURE_TO_TIME_SQUARED_ALLOWED_RATIO_TO_BEST_BY_PATH_LENGTH[ptr_path_len]
    ):
        return
    pressure_per_tick = sum([vertices[x].flow_rate for x in ptr.opened_valves])
    possible_pressure = ptr.pressure_released + ptr.time_remaining * pressure_per_tick
    if ptr.p1_time_to_dest < ptr.time_remaining:
        possible_pressure += (ptr.time_remaining - ptr.p1_time_to_dest) * vertices[
            ptr.p1_dest
        ].flow_rate
    if ptr.p2_time_to_dest < ptr.time_remaining:
        possible_pressure += (ptr.time_remaining - ptr.p2_time_to_dest) * vertices[
            ptr.p2_dest
        ].flow_rate

    if possible_pressure > BEST_YET:
        BEST_YET = possible_pressure
        BEST_PTR = BfsPtr(
            time_remaining=ptr.time_remaining,
            opened_valves=ptr.opened_valves,
            pressure_released=possible_pressure,
            p1_dest=ptr.p1_dest,
            p1_time_to_dest=ptr.p1_time_to_dest,
            p2_dest=ptr.p2_dest,
            p2_time_to_dest=ptr.p2_time_to_dest,
            path=ptr.path,
        )
    for vertex in non_zero_vertices:
        if (
            vertex in ptr.opened_valves
            or ptr.p1_dest == vertex
            or ptr.p2_dest == vertex
        ):
            continue
        dt = len(tunnels_map[ptr.p1_dest][vertex].path_to_now) + 1
        candidate_child = BfsPtr(
            time_remaining=ptr.time_remaining - ptr.p1_time_to_dest,
            opened_valves=sorted(ptr.opened_valves + [ptr.p1_dest]),
            p1_dest=vertex,
            p1_time_to_dest=dt,
            p2_dest=ptr.p2_dest,
            p2_time_to_dest=ptr.p2_time_to_dest - ptr.p1_time_to_dest,
            pressure_released=ptr.pressure_released
            + pressure_per_tick * ptr.p1_time_to_dest,
            path=ptr.path + [(ptr.p1_dest, ptr.time_remaining - ptr.p1_time_to_dest)],
        )
        if candidate_child.time_remaining < 0:
            continue
        path_len = len(candidate_child.path)
        if path_len >= PRESSURE_TO_TIME_MIN_PATH_LENGTH:
            pressure_to_time = candidate_child.pressure_released / (
                (MAX_TIME - candidate_child.time_remaining) ** TIME_EXPONENT
            )
            if (
                path_len in BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH
                and pressure_to_time / BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH[path_len]
                < PRESSURE_TO_TIME_SQUARED_ALLOWED_RATIO_TO_BEST_BY_PATH_LENGTH[
                    path_len
                ]
            ):
                continue
            if (
                path_len not in BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH
                or pressure_to_time > BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH[path_len]
            ):
                BEST_PRESSURE_TO_TIME_BY_PATH_LENGTH[path_len] = pressure_to_time
        if candidate_child.p1_time_to_dest > candidate_child.p2_time_to_dest:
            tmp_time = candidate_child.p1_time_to_dest
            candidate_child.p1_time_to_dest = candidate_child.p2_time_to_dest
            candidate_child.p2_time_to_dest = tmp_time
            tmp_dest = candidate_child.p1_dest
            candidate_child.p1_dest = ptr.p2_dest
            candidate_child.p2_dest = tmp_dest
        fingerprint = ptr_fingerprint(candidate_child)
        if fingerprint not in BFS_DICT:
            BFS_DICT[fingerprint] = [candidate_child]
        else:
            child_handled = False
            for i, existing_ptr in list(enumerate(BFS_DICT[fingerprint]))[::-1]:
                if is_better_than(existing_ptr, candidate_child):
                    child_handled = True
                    break
                if is_better_than(candidate_child, existing_ptr):
                    BFS_DICT[fingerprint] = (
                        BFS_DICT[fingerprint][:i] + BFS_DICT[fingerprint][i + 1 :]
                    )
            if not child_handled:
                BFS_DICT[fingerprint].append(candidate_child)


def choose_2(arr: list[str]) -> list[tuple[str, str]]:
    results: list[tuple[str, str]] = []
    for i in range(len(arr) - 1):
        for j in range(i + 1, len(arr)):
            results.append((arr[i], arr[j]))
    return results


T = TypeVar("T")


def flatten(l: list[list[T]]) -> list[T]:
    return [item for sublist in l for item in sublist]


def solve() -> tuple[int, Optional[BfsPtr]]:
    global BFS_DICT
    vertices = parse_file("input.txt")
    non_zero_vertices = [
        vertex.name for vertex in vertices.values() if vertex.flow_rate > 0
    ]
    tunnels_map = create_map(vertices)
    start_ptrs = []
    for p1, p2 in choose_2(non_zero_vertices):
        p1_distance = len(tunnels_map["AA"][p1].path_to_now)
        p2_distance = len(tunnels_map["AA"][p2].path_to_now)
        if p2_distance < p1_distance:
            tmp = p1
            p1 = p2
            p2 = tmp
            tmp_distance = p1_distance
            p1_distance = p2_distance
            p2_distance = tmp_distance
        start_ptrs.append(
            BfsPtr(
                time_remaining=MAX_TIME,
                p1_dest=p1,
                p1_time_to_dest=p1_distance + 1,
                p2_dest=p2,
                p2_time_to_dest=p2_distance + 1,
                opened_valves=[],
                path=[],
                pressure_released=0,
            )
        )
    BFS_DICT = {ptr_fingerprint(start_ptr): [start_ptr] for start_ptr in start_ptrs}
    while len(BFS_DICT) > 0:
        copy_of_bfs_dict = {**BFS_DICT}
        BFS_DICT = {}
        ptrs = flatten(cast(list[list[BfsPtr]], map(list, copy_of_bfs_dict.values())))
        random.shuffle(ptrs)
        for ptr in tqdm(ptrs):
            get_bfs_kids(
                vertices=vertices,
                tunnels_map=tunnels_map,
                ptr=ptr,
                non_zero_vertices=non_zero_vertices,
            )
    return BEST_YET, BEST_PTR


if __name__ == "__main__":
    print(solve())
