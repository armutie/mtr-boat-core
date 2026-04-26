from __future__ import annotations

from math import hypot

from .config import NavConfig
from .models import RadarCluster, Zone


def clamp(value: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return min(max(value, lo), hi)


def _mean(values: list[float], fallback: float = 0.0) -> float:
    return sum(values) / len(values) if values else fallback


def _distance(a: dict, b: dict) -> float:
    return hypot(float(a.get("x", 0.0)) - float(b.get("x", 0.0)), float(a.get("y", 0.0)) - float(b.get("y", 0.0)))


def cluster_zone(cx: float, cy: float, cfg: NavConfig) -> Zone:
    _ = cy
    if abs(cx) <= cfg.front_half_width:
        return "front"
    if cx < -cfg.left_right_deadband:
        return "left"
    if cx > cfg.left_right_deadband:
        return "right"
    return "unknown"


def compute_cluster_confidence(points: list[dict], cfg: NavConfig, is_singleton: bool) -> float:
    cy = _mean([float(p.get("y", 0.0)) for p in points])
    count = len(points)
    snrs = [float(p["snr_raw"]) for p in points if "snr_raw" in p]
    mean_snr = _mean(snrs) if snrs else None

    distance_weight = clamp(1.0 / max(cy, 0.3))
    count_weight = clamp(count / 3.0)
    if mean_snr is None:
        snr_weight = 0.6
    else:
        snr_weight = clamp((mean_snr - 80.0) / 220.0, 0.2, 1.0)
    base_weight = cfg.singleton_weight if is_singleton else cfg.cluster_weight
    return clamp(base_weight * distance_weight * count_weight * snr_weight)


def _make_cluster(points: list[dict], cfg: NavConfig, is_singleton: bool) -> RadarCluster:
    xs = [float(p.get("x", 0.0)) for p in points]
    ys = [float(p.get("y", 0.0)) for p in points]
    zs = [float(p.get("z", 0.0)) for p in points]
    dopplers = [float(p.get("doppler", 0.0)) for p in points]
    snrs = [float(p["snr_raw"]) for p in points if "snr_raw" in p]

    cx = _mean(xs)
    cy = _mean(ys)
    zone = cluster_zone(cx, cy, cfg)
    confidence = compute_cluster_confidence(points, cfg, is_singleton)
    return RadarCluster(
        points=points,
        cx=cx,
        cy=cy,
        cz=_mean(zs),
        mean_doppler=_mean(dopplers),
        mean_snr_raw=_mean(snrs) if snrs else None,
        count=len(points),
        is_singleton=is_singleton,
        confidence=confidence,
        zone=zone,
    )


def cluster_points(points: list[dict], cfg: NavConfig) -> list[RadarCluster]:
    if not points:
        return []

    visited: set[int] = set()
    assigned: set[int] = set()
    clusters: list[list[int]] = []

    def region_query(index: int) -> list[int]:
        return [i for i, other in enumerate(points) if _distance(points[index], other) <= cfg.cluster_eps_m]

    for index in range(len(points)):
        if index in visited:
            continue
        visited.add(index)
        neighbors = region_query(index)
        if len(neighbors) < cfg.cluster_min_points:
            continue

        cluster = set(neighbors)
        assigned.update(neighbors)
        queue = list(neighbors)
        while queue:
            candidate = queue.pop()
            if candidate not in visited:
                visited.add(candidate)
                candidate_neighbors = region_query(candidate)
                if len(candidate_neighbors) >= cfg.cluster_min_points:
                    for neighbor in candidate_neighbors:
                        if neighbor not in cluster:
                            queue.append(neighbor)
                            cluster.add(neighbor)
            assigned.add(candidate)
        clusters.append(sorted(cluster))

    output = [_make_cluster([points[i] for i in cluster], cfg, is_singleton=False) for cluster in clusters]

    if cfg.keep_singletons:
        for index, point in enumerate(points):
            if index not in assigned:
                output.append(_make_cluster([point], cfg, is_singleton=True))

    output.sort(key=lambda cluster: (cluster.cy, abs(cluster.cx)))
    return output


def clusters_to_evidence(clusters: list[RadarCluster], cfg: NavConfig) -> tuple[float, float, float]:
    _ = cfg
    left = 0.0
    front = 0.0
    right = 0.0
    for cluster in clusters:
        if cluster.zone == "left":
            left += cluster.confidence
        elif cluster.zone == "front":
            front += cluster.confidence
        elif cluster.zone == "right":
            right += cluster.confidence
    return clamp(left), clamp(front), clamp(right)


def clusters_to_emergency_evidence(clusters: list[RadarCluster], cfg: NavConfig) -> float:
    strongest = 0.0
    for cluster in clusters:
        if abs(cluster.cx) > cfg.emergency_center_half_width:
            continue
        if cluster.cy > cfg.emergency_near_y:
            continue

        distance_ratio = (cluster.cy - cfg.min_y) / max(cfg.emergency_near_y - cfg.min_y, 0.01)
        proximity = 1.0 - 0.4 * clamp(distance_ratio, 0.0, 1.0)
        strongest = max(strongest, cluster.confidence * proximity)
    return clamp(strongest)
