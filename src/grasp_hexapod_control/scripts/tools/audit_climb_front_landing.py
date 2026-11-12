#!/usr/bin/env python3
"""Offline CAD landing-patch audit; geometry diagnostics only, never feedback."""
import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


def _edge_key(first, second):
    return tuple(sorted((tuple(first), tuple(second))))


def _point_segment_distance(point, start, end):
    direction = end - start
    denominator = float(np.dot(direction, direction))
    if denominator <= 1e-18:
        return float(np.linalg.norm(point - start))
    fraction = np.clip(np.dot(point - start, direction) / denominator, 0.0, 1.0)
    return float(np.linalg.norm(point - (start + fraction * direction)))


def _point_rectangle_distance(point, minimum, maximum):
    return float(np.linalg.norm(np.maximum(np.maximum(minimum - point, 0.0), point - maximum)))


def _orientation(first, second, third):
    return float(np.cross(second - first, third - first))


def _on_segment(first, second, point):
    return (min(first[0], second[0]) - 1e-12 <= point[0] <= max(first[0], second[0]) + 1e-12
            and min(first[1], second[1]) - 1e-12 <= point[1] <= max(first[1], second[1]) + 1e-12)


def _segments_intersect(first, second, third, fourth):
    values = (_orientation(first, second, third), _orientation(first, second, fourth),
              _orientation(third, fourth, first), _orientation(third, fourth, second))
    if ((values[0] > 0) != (values[1] > 0) and (values[2] > 0) != (values[3] > 0)):
        return True
    return ((abs(values[0]) <= 1e-12 and _on_segment(first, second, third))
            or (abs(values[1]) <= 1e-12 and _on_segment(first, second, fourth))
            or (abs(values[2]) <= 1e-12 and _on_segment(third, fourth, first))
            or (abs(values[3]) <= 1e-12 and _on_segment(third, fourth, second)))


def _segment_rectangle_distance(start, end, minimum, maximum):
    if np.all((start >= minimum) & (start <= maximum)) or np.all((end >= minimum) & (end <= maximum)):
        return 0.0
    corners = np.array(((minimum[0], minimum[1]), (maximum[0], minimum[1]),
                        (maximum[0], maximum[1]), (minimum[0], maximum[1])))
    edges = tuple((corners[index], corners[(index + 1) % 4]) for index in range(4))
    if any(_segments_intersect(start, end, first, second) for first, second in edges):
        return 0.0
    return min([_point_rectangle_distance(start, minimum, maximum),
                _point_rectangle_distance(end, minimum, maximum)]
               + [_point_segment_distance(corner, start, end) for corner in corners])


class PatchClassifier:
    """Exact closest faces plus connected, positively oriented coplanar patches."""

    def __init__(self, world_triangles):
        self.triangles = np.asarray(world_triangles, dtype=float)
        if self.triangles.ndim != 3 or self.triangles.shape[1:] != (3, 3):
            raise ValueError("world triangles must have shape (N,3,3)")
        raw = np.cross(self.triangles[:, 1] - self.triangles[:, 0],
                       self.triangles[:, 2] - self.triangles[:, 0])
        self.areas = .5 * np.linalg.norm(raw, axis=1)
        if np.any(self.areas <= 1e-15):
            raise ValueError("degenerate landing triangle")
        self.normals = raw / (2.0 * self.areas[:, None])
        self.edges = defaultdict(list)
        for index, triangle in enumerate(np.round(self.triangles, 9)):
            for first, second in ((0, 1), (1, 2), (2, 0)):
                self.edges[_edge_key(triangle[first], triangle[second])].append(index)

    def closest(self, point):
        """Vectorized Ericson closest-point query, returning face and point."""
        point = np.asarray(point, dtype=float)
        a, b, c = self.triangles[:, 0], self.triangles[:, 1], self.triangles[:, 2]
        ab, ac, ap = b-a, c-a, point-a
        d1, d2 = np.einsum("ij,ij->i", ab, ap), np.einsum("ij,ij->i", ac, ap)
        closest, done = np.empty_like(a), np.zeros(len(a), dtype=bool)
        mask = (d1 <= 0) & (d2 <= 0); closest[mask] = a[mask]; done |= mask
        bp = point-b; d3, d4 = np.einsum("ij,ij->i", ab, bp), np.einsum("ij,ij->i", ac, bp)
        mask = (~done) & (d3 >= 0) & (d4 <= d3); closest[mask] = b[mask]; done |= mask
        vc = d1*d4-d3*d2; mask = (~done) & (vc <= 0) & (d1 >= 0) & (d3 <= 0)
        weight = np.zeros(len(a)); weight[mask] = d1[mask] / (d1[mask]-d3[mask]); closest[mask] = a[mask]+weight[mask,None]*ab[mask]; done |= mask
        cp = point-c; d5, d6 = np.einsum("ij,ij->i", ab, cp), np.einsum("ij,ij->i", ac, cp)
        mask = (~done) & (d6 >= 0) & (d5 <= d6); closest[mask] = c[mask]; done |= mask
        vb = d5*d2-d1*d6; mask = (~done) & (vb <= 0) & (d2 >= 0) & (d6 <= 0)
        weight = np.zeros(len(a)); weight[mask] = d2[mask] / (d2[mask]-d6[mask]); closest[mask] = a[mask]+weight[mask,None]*ac[mask]; done |= mask
        va = d3*d6-d5*d4; mask = (~done) & (va <= 0) & ((d4-d3) >= 0) & ((d5-d6) >= 0)
        weight = np.zeros(len(a)); weight[mask] = (d4[mask]-d3[mask]) / ((d4[mask]-d3[mask])+(d5[mask]-d6[mask])); closest[mask] = b[mask]+weight[mask,None]*(c[mask]-b[mask]); done |= mask
        mask = ~done; indices = np.flatnonzero(mask); denominator = va[mask]+vb[mask]+vc[mask]; regular = np.abs(denominator) > 1e-15
        face = indices[regular]; v, w = vb[face]/denominator[regular], vc[face]/denominator[regular]
        closest[face] = a[face]+v[:,None]*ab[face]+w[:,None]*ac[face]
        closest[indices[~regular]] = a[indices[~regular]]
        distances = np.linalg.norm(closest-point, axis=1); index = int(np.argmin(distances))
        return index, closest[index], float(distances[index])

    def _component(self, seed):
        normal, origin = self.normals[seed], self.triangles[seed, 0]
        component, pending = {seed}, [seed]
        while pending:
            current = pending.pop()
            triangle = np.round(self.triangles[current], 9)
            for first, second in ((0, 1), (1, 2), (2, 0)):
                for neighbor in self.edges[_edge_key(triangle[first], triangle[second])]:
                    coplanar = (float(np.dot(self.normals[neighbor], normal)) >= .999999
                                and np.max(np.abs((self.triangles[neighbor] - origin) @ normal)) <= 1e-6)
                    if coplanar and neighbor not in component:
                        component.add(neighbor); pending.append(neighbor)
        return frozenset(component)

    def _boundary(self, component):
        boundary = []
        for current in component:
            triangle = np.round(self.triangles[current], 9)
            for first, second in ((0, 1), (1, 2), (2, 0)):
                edge = _edge_key(triangle[first], triangle[second])
                if sum(owner in component for owner in self.edges[edge]) == 1:
                    boundary.append(np.asarray(edge, dtype=float))
        return boundary

    def _contains_xy(self, component, xy):
        for index in component:
            triangle = self.triangles[index]
            matrix = np.column_stack((triangle[1, :2]-triangle[0, :2], triangle[2, :2]-triangle[0, :2]))
            determinant = np.linalg.det(matrix)
            if abs(determinant) <= 1e-12:
                continue
            uv = np.linalg.solve(matrix, np.asarray(xy, dtype=float)-triangle[0, :2])
            if np.all(uv >= -1e-9) and uv.sum() <= 1.0+1e-9:
                return True
        return False

    def describe(self, point, sphere_gap_m=None):
        index, nearest, distance = self.closest(point)
        component = self._component(index)
        boundary = self._boundary(component)
        component_hash = hashlib.sha256(np.asarray(sorted(component), dtype=np.int64).tobytes()).hexdigest()
        return {
            "nearest_triangle_index": index, "nearest_point_world_xyz_m": nearest.tolist(),
            "face_normal_world": self.normals[index].tolist(), "face_normal_z": float(self.normals[index, 2]),
            "distance_to_stl_m": distance, "component_indices": component,
            "component_hash": component_hash, "component_triangle_count": len(component),
            "component_area_m2": float(self.areas[list(component)].sum()), "boundary": boundary,
            "center_in_component_xy": self._contains_xy(component, np.asarray(point)[:2]),
            "sphere_gap_m": sphere_gap_m,
        }


def rectangle_patch_check(classifier, point, dx=(-.045, 0.0), dy=(-.010, .010), extra_xy_m=.001,
                          min_area_m2=.020, min_inset_m=.015, sphere_gap_m=None):
    """Conservative XY boundary-to-error-rectangle screen; holes are boundaries."""
    detail = classifier.describe(point, sphere_gap_m=sphere_gap_m)
    xy = np.asarray(point, dtype=float)[:2]
    minimum = xy + np.array((dx[0]-extra_xy_m, dy[0]-extra_xy_m))
    maximum = xy + np.array((dx[1]+extra_xy_m, dy[1]+extra_xy_m))
    boundary_distance = min(_segment_rectangle_distance(edge[0, :2], edge[1, :2], minimum, maximum)
                            for edge in detail["boundary"])
    detail.update({"rectangle_xy_min_m": minimum.tolist(), "rectangle_xy_max_m": maximum.tolist(),
                   "rectangle_boundary_distance_m": boundary_distance,
                   "positive_up_normal": detail["face_normal_z"] >= .90,
                   "large_planar_component": detail["component_area_m2"] >= min_area_m2,
                   "robust_rectangle": boundary_distance > min_inset_m,
                   "safe_patch": bool(detail["center_in_component_xy"] and detail["face_normal_z"] >= .90
                                      and detail["component_area_m2"] >= min_area_m2 and boundary_distance > min_inset_m)})
    return detail


def same_component(first, second):
    return first["component_hash"] == second["component_hash"]


def gap_in_interval(gap_m, lower_m=-.0005, upper_m=.002):
    return lower_m <= float(gap_m) <= upper_m


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--points-json", type=Path, required=True,
                        help="JSON mapping label -> [x,y,z] planned or actual foot centres")
    args = parser.parse_args()
    from build_climb_front import MESH, TRANSLATION, foot_sphere_signed_gap, load_triangles
    triangles = load_triangles(MESH) + TRANSLATION
    classifier = PatchClassifier(triangles)
    points = json.loads(args.points_json.read_text())
    report = {"method": "offline CAD robust-rectangle landing audit", "points": {},
              "boundary": "Geometry diagnostic only; never controller feedback or contact/load proof."}
    for label, point in points.items():
        point = np.asarray(point, dtype=float)
        report["points"][label] = rectangle_patch_check(
            classifier, point, sphere_gap_m=foot_sphere_signed_gap(triangles, point))
        report["points"][label].pop("component_indices")
        report["points"][label].pop("boundary")
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"report": str(args.report), "safe": {key: item["safe_patch"] for key, item in report["points"].items()}}))


if __name__ == "__main__":
    main()
