from .config import NavConfig


def filter_points(points: list[dict], cfg: NavConfig) -> list[dict]:
    filtered = []
    for point in points:
        x = float(point.get("x", 0.0))
        y = float(point.get("y", 0.0))

        if y < cfg.min_y:
            continue
        if cfg.max_y is not None and y > cfg.max_y:
            continue
        if cfg.lateral_limit is not None and abs(x) > cfg.lateral_limit:
            continue
        if (
            cfg.min_snr_raw is not None
            and "snr_raw" in point
            and point["snr_raw"] < cfg.min_snr_raw
        ):
            continue

        filtered.append(point)
    return filtered
