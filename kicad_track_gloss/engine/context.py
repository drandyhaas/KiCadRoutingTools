"""Precomputed spatial context shared by Track Gloss planning passes."""

from __future__ import annotations

from collections import defaultdict
import math

from .geometry import EPS, point_in_polygon, segment_distance
from .model import segment_key


def line_bbox(a, b, margin=0.0):
    return (min(a[0], b[0]) - margin, min(a[1], b[1]) - margin,
            max(a[0], b[0]) + margin, max(a[1], b[1]) + margin)


class SpatialIndex:
    """Small dependency-free uniform grid with exact conservative queries."""

    def __init__(self, items, bbox, cell_size):
        self.items = tuple(items)
        self.cell_size = max(float(cell_size), 0.25)
        self.cells = defaultdict(list)
        for index, item in enumerate(self.items):
            for cell in self._cells_for(bbox(item)):
                self.cells[cell].append(index)

    def _cells_for(self, bounds):
        x0, y0, x1, y1 = bounds
        scale = self.cell_size
        ix0, iy0 = math.floor(x0 / scale), math.floor(y0 / scale)
        ix1, iy1 = math.floor(x1 / scale), math.floor(y1 / scale)
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                yield ix, iy

    def query(self, bounds):
        found = set()
        for cell in self._cells_for(bounds):
            found.update(self.cells.get(cell, ()))
        return (self.items[index] for index in sorted(found))


class PlannerContext:
    """Immutable lookup data reused across candidate and fallback searches."""

    def __init__(self, model):
        self.model = model
        self.segment_by_key = {segment_key(segment): segment
                               for segment in model.segments}
        clearances = list(model.net_clearances.values())
        clearances.extend(segment.clearance for segment in model.segments
                          if segment.clearance >= 0.0)
        self.max_net_clearance = max(
            [model.minimum_clearance] + clearances + [0.0])
        self.max_segment_halfwidth = max(
            [segment.width / 2.0 for segment in model.segments] + [0.0])
        self.max_obstacle_radius = max(
            [obstacle.radius for obstacle in model.obstacles] + [0.0])
        self.max_obstacle_clearance = max(
            [obstacle.clearance for obstacle in model.obstacles] + [0.0])
        self.max_pad_radius = max(
            [math.hypot(pad.width, pad.height) / 2.0
             for pad in model.pad_regions] + [0.0])
        self.max_pad_clearance = max(
            [pad.clearance for pad in model.pad_regions] + [0.0])

        # A board-adaptive grid avoids both huge sparse dictionaries and dense
        # all-object scans. It affects performance only; exact tests still run
        # after every conservative query.
        bounds = model.board_bounds
        if bounds and model.segments:
            area = max((bounds[2] - bounds[0]) * (bounds[3] - bounds[1]), 1.0)
            cell_size = max(1.0, min(10.0, math.sqrt(area / len(model.segments))))
        else:
            cell_size = 5.0
        self.cell_size = cell_size

        self.segments = SpatialIndex(
            model.segments,
            lambda item: line_bbox(
                (item.start_x, item.start_y), (item.end_x, item.end_y)),
            cell_size)
        self.obstacles = SpatialIndex(
            model.obstacles,
            lambda item: (item.x, item.y, item.x, item.y), cell_size)
        self.pads = SpatialIndex(
            model.pad_regions,
            lambda item: (item.x, item.y, item.x, item.y), cell_size)
        self.keepouts = SpatialIndex(
            model.keepouts,
            lambda item: (
                min((point[0] for point in item.points), default=0.0),
                min((point[1] for point in item.points), default=0.0),
                max((point[0] for point in item.points), default=0.0),
                max((point[1] for point in item.points), default=0.0)),
            cell_size)
        outline = model.board_outline
        self.outline_edges = []
        self.hole_edges = []
        if outline:
            for polygon in outline.outlines:
                edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
                self.outline_edges.append(SpatialIndex(
                    edges, lambda edge: line_bbox(*edge), cell_size))
            for polygon in outline.holes:
                edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
                self.hole_edges.append(SpatialIndex(
                    edges, lambda edge: line_bbox(*edge), cell_size))

    @staticmethod
    def _near_edge(a, b, margin, edge_index):
        bounds = line_bbox(a, b, margin + EPS)
        return any(segment_distance(a, b, c, d) < margin + EPS
                   for c, d in edge_index.query(bounds))

    def segment_inside_board(self, a, b, margin=0.0):
        """Indexed equivalent of geometry.segment_inside_board()."""
        outline = self.model.board_outline
        if outline is None or not outline.outlines:
            return True
        for polygon, edges in zip(outline.outlines, self.outline_edges):
            if not (point_in_polygon(a, polygon) and
                    point_in_polygon(b, polygon)):
                continue
            if self._near_edge(a, b, margin, edges):
                continue
            blocked = False
            for hole, hole_edges in zip(outline.holes, self.hole_edges):
                if (point_in_polygon(a, hole) or point_in_polygon(b, hole) or
                        self._near_edge(a, b, margin, hole_edges)):
                    blocked = True
                    break
            if not blocked:
                return True
        return False

    def nearby_segments(self, a, b, moving_clearance, moving_width):
        margin = (max(moving_clearance, self.max_net_clearance) +
                  moving_width / 2.0 + self.max_segment_halfwidth)
        return self.segments.query(line_bbox(a, b, margin))

    def nearby_obstacles(self, a, b, moving_clearance, moving_width):
        margin = (max(moving_clearance, self.max_obstacle_clearance) +
                  moving_width / 2.0 + self.max_obstacle_radius)
        return self.obstacles.query(line_bbox(a, b, margin))

    def nearby_pads(self, a, b, moving_clearance, moving_width):
        margin = (max(moving_clearance, self.max_pad_clearance) +
                  moving_width / 2.0 + self.max_pad_radius)
        return self.pads.query(line_bbox(a, b, margin))

    def nearby_keepouts(self, a, b, clearance, moving_width):
        return self.keepouts.query(line_bbox(
            a, b, clearance + moving_width / 2.0))
