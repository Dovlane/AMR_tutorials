from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple

import numpy


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class LineSegment:
    rho: float
    alpha: float
    start_point: Point2D
    end_point: Point2D
    point_count: int
    max_error: float
    start_index: int
    end_index: int


@dataclass(frozen=True)
class FittedLine:
    rho: float
    alpha: float
    max_error: float
    distances: numpy.ndarray


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def point_chunks_from_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_point_gap: float,
    min_points: int,
) -> List[numpy.ndarray]:
    chunks: List[numpy.ndarray] = []
    current_points: List[Point2D] = []
    previous_point: numpy.ndarray | None = None

    valid_range_min = max(range_min, 0.0)
    valid_range_max = range_max if range_max > 0.0 else math.inf

    def flush_current_points() -> None:
        nonlocal current_points, previous_point
        if len(current_points) >= min_points:
            chunks.append(numpy.asarray(current_points, dtype=float))
        current_points = []
        previous_point = None

    for index, scan_range in enumerate(ranges):
        if not math.isfinite(scan_range) or not (
            valid_range_min <= scan_range <= valid_range_max
        ):
            flush_current_points()
            continue

        angle = angle_min + index * angle_increment
        point = numpy.array(
            [scan_range * math.cos(angle), scan_range * math.sin(angle)],
            dtype=float,
        )

        if (
            previous_point is not None
            and max_point_gap > 0.0
            and numpy.linalg.norm(point - previous_point) > max_point_gap
        ):
            flush_current_points()

        current_points.append((float(point[0]), float(point[1])))
        previous_point = point

    flush_current_points()
    return chunks


def fit_line(points: numpy.ndarray) -> FittedLine:
    if len(points) < 2:
        raise ValueError("At least two points are required to fit a line.")

    centroid = numpy.mean(points, axis=0)
    centered_points = points - centroid
    _, _, vh = numpy.linalg.svd(centered_points, full_matrices=False)
    normal = vh[-1]

    rho = float(numpy.dot(centroid, normal))
    if rho < 0.0:
        rho = -rho
        normal = -normal

    alpha = normalize_angle(math.atan2(float(normal[1]), float(normal[0])))
    distances = numpy.abs(points @ normal - rho)
    max_error = float(numpy.max(distances)) if len(distances) else 0.0

    return FittedLine(rho=rho, alpha=alpha, max_error=max_error, distances=distances)


def split_and_merge_iterative(
    points: numpy.ndarray,
    split_threshold: float,
    merge_threshold: float,
    min_points: int,
) -> List[LineSegment]:
    if len(points) < min_points:
        return []

    pending_segments = [(0, len(points) - 1)]
    split_segments: List[LineSegment] = []

    while pending_segments:
        start_index, end_index = pending_segments.pop()
        split_index = _find_split_index(
            points,
            start_index,
            end_index,
            split_threshold,
            min_points,
        )

        if split_index is None:
            split_segments.append(_make_segment(points, start_index, end_index))
            continue

        pending_segments.append((split_index, end_index))
        pending_segments.append((start_index, split_index))

    split_segments.sort(key=lambda segment: (segment.start_index, segment.end_index))
    return _merge_segments(points, split_segments, merge_threshold)


def extract_lines(
    point_chunks: Iterable[numpy.ndarray],
    split_threshold: float,
    merge_threshold: float,
    min_points: int,
) -> List[LineSegment]:
    lines: List[LineSegment] = []
    for points in point_chunks:
        lines.extend(
            split_and_merge_iterative(
                points,
                split_threshold=split_threshold,
                merge_threshold=merge_threshold,
                min_points=min_points,
            )
        )
    return lines


def _find_split_index(
    points: numpy.ndarray,
    start_index: int,
    end_index: int,
    split_threshold: float,
    min_points: int,
) -> int | None:
    segment_points = points[start_index : end_index + 1]
    if len(segment_points) < 2 * min_points - 1:
        return None

    fitted_line = fit_line(segment_points)
    first_valid_split_index = min_points - 1
    last_valid_split_index = len(segment_points) - min_points
    valid_distances = fitted_line.distances[
        first_valid_split_index : last_valid_split_index + 1
    ]
    relative_split_index = int(numpy.argmax(valid_distances)) + first_valid_split_index
    absolute_split_index = start_index + relative_split_index

    left_count = absolute_split_index - start_index + 1
    right_count = end_index - absolute_split_index + 1
    if (
        fitted_line.distances[relative_split_index] <= split_threshold
        or left_count < min_points
        or right_count < min_points
    ):
        return None

    return absolute_split_index


def _merge_segments(
    points: numpy.ndarray,
    segments: Sequence[LineSegment],
    merge_threshold: float,
) -> List[LineSegment]:
    if not segments:
        return []

    merged_segments: List[LineSegment] = []
    current_segment = segments[0]

    for next_segment in segments[1:]:
        combined_start = current_segment.start_index
        combined_end = next_segment.end_index
        combined_line = fit_line(points[combined_start : combined_end + 1])

        if combined_line.max_error <= merge_threshold:
            current_segment = _make_segment(points, combined_start, combined_end)
        else:
            merged_segments.append(current_segment)
            current_segment = next_segment

    merged_segments.append(current_segment)
    return merged_segments


def _make_segment(
    points: numpy.ndarray,
    start_index: int,
    end_index: int,
) -> LineSegment:
    segment_points = points[start_index : end_index + 1]
    fitted_line = fit_line(segment_points)
    start_point, end_point = _project_segment_endpoints(
        fitted_line.rho,
        fitted_line.alpha,
        segment_points[0],
        segment_points[-1],
    )

    return LineSegment(
        rho=fitted_line.rho,
        alpha=fitted_line.alpha,
        start_point=start_point,
        end_point=end_point,
        point_count=len(segment_points),
        max_error=fitted_line.max_error,
        start_index=start_index,
        end_index=end_index,
    )


def _project_segment_endpoints(
    rho: float,
    alpha: float,
    raw_start_point: numpy.ndarray,
    raw_end_point: numpy.ndarray,
) -> Tuple[Point2D, Point2D]:
    normal = numpy.array([math.cos(alpha), math.sin(alpha)], dtype=float)
    direction = numpy.array([-math.sin(alpha), math.cos(alpha)], dtype=float)
    point_on_line = rho * normal

    start_distance = float(numpy.dot(raw_start_point - point_on_line, direction))
    end_distance = float(numpy.dot(raw_end_point - point_on_line, direction))

    start_point = point_on_line + start_distance * direction
    end_point = point_on_line + end_distance * direction

    return (
        (float(start_point[0]), float(start_point[1])),
        (float(end_point[0]), float(end_point[1])),
    )
