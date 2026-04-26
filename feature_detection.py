import math
from collections import deque
from typing import Set

import networkx as nx


def grow_region(
    G: nx.Graph,
    seed_face_id: int,
    angle_threshold: float = 40.0,
    machining_direction: tuple = (0.0, 0.0, 1.0),
    parallel_angle_tol: float = 5.0,
) -> Set[int]:

    def is_plane(node_id: int) -> bool:
        return G.nodes[node_id].get("surface_type") == "Plane"

    def is_parallel_to_direction(node_id: int) -> bool:
        normal = G.nodes[node_id].get("normal")
        if normal is None:
            return False
        nx_, ny_, nz_ = normal
        dx, dy, dz = machining_direction

        n_mag = math.sqrt(nx_**2 + ny_**2 + nz_**2)
        d_mag = math.sqrt(dx**2 + dy**2 + dz**2)
        if n_mag < 1e-10 or d_mag < 1e-10:
            return False

        dot = (nx_ * dx + ny_ * dy + nz_ * dz) / (n_mag * d_mag)
        dot = max(-1.0, min(1.0, dot))        # clamp for floating-point safety
        angle_deg = math.degrees(math.acos(abs(dot)))  # always in [0°, 90°]

        return angle_deg <= parallel_angle_tol

    region: Set[int] = set()
    queue: deque[int] = deque([seed_face_id])
    while queue:
        current = queue.popleft()
        if current in region:
            continue

        if is_plane(current):
            if not is_parallel_to_direction(current):
                continue
            neighbors = list(G.neighbors(current))
            if any(is_plane(nbr) for nbr in neighbors) or not neighbors:
                continue
        region.add(current)
        G.nodes[current]["flag"] = True

        for nbr in G.neighbors(current):
            if nbr in region:
                continue
            angle = G.edges[current, nbr].get("angle_deg", 0.0)
            if math.isnan(angle) or angle > angle_threshold:
                continue
            queue.append(nbr)

    return region
