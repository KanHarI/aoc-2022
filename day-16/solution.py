import re
from dataclasses import dataclass

LINE_REGEX = re.compile(
    "^Valve ([A-Z]{2}) has flow rate=([0-9]+); tunnel(?:s)? lead(?:s)? to valve(?:s)? ((?:[A-Z]{2}(?:, )?)+)$"
)


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


def ptr_fingerprint(ptr: BfsPtr):
    return "".join(ptr.opened_valves) + "_" + str(ptr.position)


def get_bfs_kids(
    vertices: dict[str, Vertex],
    tunnels_map: dict[str, dict[str, MapTracer]],
    ptr: BfsPtr,
    non_zero_vertices: list[str],
):
    pass


def solve() -> None:
    vertices = parse_file("input.txt")
    non_zero_vertices = [
        vertex.name for vertex in vertices.values() if vertex.flow_rate > 0
    ]
    tunnels_map = create_map(vertices)


if __name__ == "__main__":
    print(solve())
