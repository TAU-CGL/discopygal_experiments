import math
from itertools import combinations

import networkx as nx

from discopygal.solvers_infra import RobotDisc, RobotPolygon, ObstaclePolygon, PathPoint, Path, PathCollection
from discopygal.solvers_infra.Solver import Solver
from discopygal.geometry_utils import bounding_boxes, collision_detection, conversions
from discopygal.bindings import FT, Point_2, Ker, Arr_trapezoid_ric_point_location, Arr_overlay_function_traits, \
    Arrangement_2, Polygon_2, Aos2, Curve_2, Ms2, Vertex, Segment_2, Halfedge, Face, TPoint


int_list = list(range(1000))


class FaceData:
    FREE = 0
    INVALID = 1

    def __init__(self, status, obstacle_ids):
        self.status = status
        self.obstacle_ids = obstacle_ids

    @staticmethod
    def valid_face():
        return FaceData(FaceData.FREE, [])

    @staticmethod
    def invalid_face(obstacle_id):
        return FaceData(FaceData.INVALID, [obstacle_id])

    @classmethod
    def merge(cls, first, second):
        if first.status == cls.INVALID or second.status == cls.INVALID:
            merged_ids = list(set(first.obstacle_ids + second.obstacle_ids))
            return FaceData(cls.INVALID, merged_ids)
        return FaceData.valid_face()


def distance(p1, p2):
    x1, y1 = p1.x().to_double(), p1.y().to_double()
    x2, y2 = p2.x().to_double(), p2.y().to_double()
    return math.hypot(x2 - x1, y2 - y1)


def is_free_face(face):
    return face.data().status == 0


def is_overlapping_faces(first, second):
    first_ids = first.data().obstacle_ids
    second_ids = second.data().obstacle_ids

    for item in first_ids:
        if item in second_ids:
            return True
    return False


def is_walkable_edge(edge):
    first_face = edge.face()
    second_face = edge.twin().face()

    if is_free_face(first_face) or is_free_face(second_face):
        return True
    return not is_overlapping_faces(first_face, second_face)


class SemiPathSolver(Solver):
    def __init__(self, eps=0.0001, **kwargs):
        super().__init__(**kwargs)
        self.eps = eps

        self.G = None
        self.robot = None
        self.arr = None
        self.pl = None
        self.conn_graph = None
        self.vertex_dict = None
        self.face_dict = None
        self.collision_detection = None

    def load_scene(self, scene):
        super().load_scene(scene)

        if len(self.scene.robots) != 1:
            raise(Exception('Unsupported number of robots'))
        if type(self.scene.robots[0]) not in [RobotDisc, RobotPolygon]:
            raise(Exception('Only disc/polygon robots are supported'))
        self.robot = self.scene.robots[0]

        if self._bounding_box is None: # If no bounding box, create one
            self._bounding_box = bounding_boxes.calc_scene_bounding_box(self.scene)

        self.arr = self.construct_cspace()
        self.arr = self.vertical_decomposition(self.arr)
        self.conn_graph, self.vertex_dict, self.face_dict = self.connectivity_graph(self.arr)
        self.collision_detection = collision_detection.ObjectCollisionDetection(self.scene.obstacles, self.robot, FT(0.0))
        self.pl = Arr_trapezoid_ric_point_location(self.arr)

    def get_arrangement(self):
        return self.arr

    def _solve(self):
        source_idx = self.find_face_index(self.robot.start, self.pl)
        target_idx = self.find_face_index(self.robot.end, self.pl)

        if source_idx < 0:
            if self.verbose:
                print("Could not find origin face.", file=self.writer)
            return PathCollection()
        if target_idx < 0:
            if self.verbose:
                print("Could not find target face.", file=self.writer)
            return PathCollection()

        if source_idx == target_idx:
            # If we are very close, then linear interpolation is enough
            return PathCollection({self.robot: Path([PathPoint(self.robot.start), PathPoint(self.robot.end)])})

        try:
            # connect source and target points to all vertexes in their faces
            source_vertexes = self.face_dict[source_idx]
            target_vertexes = self.face_dict[target_idx]

            start_idx = -100
            end_idx = -200
            self.conn_graph.add_node(start_idx)
            self.conn_graph.add_node(end_idx)

            for sv in source_vertexes:
                self.conn_graph.add_edge(start_idx, sv)
            for tv in target_vertexes:
                self.conn_graph.add_edge(end_idx, tv)

            g_path = nx.shortest_path(self.conn_graph, start_idx, end_idx)
        except nx.exception.NetworkXNoPath:
            if self.verbose:
                print("Could not find a path from source to dest", file=self.writer)
            return PathCollection()

        # Convert g_path to a linear path for the robot
        path = self.find_valid_path(g_path, self.robot.start, self.robot.end, self.vertex_dict)
        return PathCollection({self.robot: path})

    @classmethod
    def get_arguments(cls):
        print("get_arguments")
        args = {
            'eps': ('epsilon for approximated offset:', 0.0001, float),
        }
        args.update(super().get_arguments())
        return args

    def construct_cspace(self):
        """
        Get the (CGAL Polygonal) obstacles and the radius of the robot,
        and construct the expanded CSPACE arrangement (also with bounding box walls)
        """
        traits = Arr_overlay_function_traits(lambda x, y: FaceData.merge(x, y))

        # Compute an arrangement for each single Minkowski sum
        arrangements = []
        for idx, obstacle in enumerate(self.scene.obstacles):
            if type(obstacle) is ObstaclePolygon:
                arr = Arrangement_2()
                if isinstance(self.robot, RobotPolygon):
                    minus_robot = Polygon_2([Point_2(-p.x(), -p.y()) for p in self.robot.poly.vertices()])
                    if minus_robot.is_clockwise_oriented():
                        minus_robot.reverse_orientation()
                    if obstacle.poly.is_clockwise_oriented():
                        obstacle.poly.reverse_orientation()
                    ms = Ms2.minkowski_sum_2(obstacle.poly, minus_robot)
                    Aos2.insert(arr, [Curve_2(edge) for edge in conversions.to_list(ms.outer_boundary().edges())])
                    for hole in ms.holes():
                        Aos2.insert(arr, [Curve_2(edge) for edge in hole.edges()])
                elif isinstance(self.robot, RobotDisc):
                    ms = Ms2.approximated_offset_2(obstacle.poly, self.robot.radius, self.eps)
                    Aos2.insert(arr, conversions.to_list(ms.outer_boundary().curves()))
                    for hole in ms.holes():
                        Aos2.insert(arr, conversions.to_list(hole.curves()))

                # Data of face: 0 - free, 1 - invalid
                ubf = arr.unbounded_face()
                ubf.set_data(FaceData.valid_face())
                invalid_face = next(next(ubf.inner_ccbs())).twin().face()
                invalid_face.set_data(FaceData.invalid_face(idx))
                for ccb in invalid_face.inner_ccbs():
                    valid_face = next(ccb).twin().face()
                    valid_face.set_data(FaceData.valid_face())

                arrangements.append(arr)

        # Overlay the arrangement
        initial = Arrangement_2()
        ubf = initial.unbounded_face()
        ubf.set_data(FaceData.valid_face())
        arrangements.insert(0, initial)
        arr = initial
        for i in range(len(arrangements)-1):
            arr = Aos2.overlay(arrangements[i], arrangements[i+1], traits)
            arrangements[i+1] = arr

        # Compute the bounding box of the arrangement and add it as edges
        min_x, max_x, min_y, max_y = self._bounding_box
        bb_arr = Aos2.Arrangement_2()
        Aos2.insert(bb_arr, [
            Curve_2(Point_2(min_x, min_y), Point_2(max_x, min_y)),
            Curve_2(Point_2(max_x, min_y), Point_2(max_x, max_y)),
            Curve_2(Point_2(max_x, max_y), Point_2(min_x, max_y)),
            Curve_2(Point_2(min_x, max_y), Point_2(min_x, min_y)),
        ])
        for face in bb_arr.faces():
            if face.is_unbounded():
                face.set_data(FaceData.invalid_face(-1))
            else:
                face.set_data(FaceData.valid_face())

        cspace = Aos2.overlay(arr, bb_arr, traits)

        return cspace

    def to_ker_point_2(self, point: Point_2):
        """
        Convert TPoint() to Ker.Point_2()
        """
        x, y = point.x(), point.y()
        # assert (not x.is_extended())
        # assert (not y.is_extended())
        return Ker.Point_2(x.a0(), y.a0())

    def vertical_decomposition(self, arr):
        """
        Take an arrangement and add edges to it that represent the vertical decomposition
        """
        lst = Aos2.decompose(arr)
        vertical_walls = []
        for pair in lst:
            v, objects = pair
            for object in objects:
                v_point = self.to_ker_point_2(v.point())
                if type(object) == Vertex:
                    v_other = self.to_ker_point_2(object.point())
                    wall = Curve_2(Segment_2(v_point, v_other))
                    vertical_walls.append(wall)
                if type(object) == Halfedge:
                    line = Ker.Line_2(self.to_ker_point_2(object.source().point()), self.to_ker_point_2(object.target().point()))
                    y_at_x = line.y_at_x(v_point.x())
                    wall = Curve_2(Segment_2(v_point, Point_2(v_point.x(), y_at_x)))
                    vertical_walls.append(wall)

        # Create an arrangement of vertical walls and overlay it
        walls_arr = Aos2.Arrangement_2()
        Aos2.insert(walls_arr, vertical_walls)
        for face in walls_arr.faces():
            face.set_data(FaceData.valid_face())

        traits = Arr_overlay_function_traits(lambda x, y: FaceData.merge(x, y))
        res = Aos2.overlay(arr, walls_arr, traits)
        return res

    def connectivity_graph(self, arr):
        """
        Get the connectivity graph from a vertical decomposition arrangement
        """
        conn_graph = nx.Graph()

        # map vertex id to vertex object
        vertex_dict = {}

        # map face id to vertexes in it
        face_dict = {}

        assert arr.is_valid()
        idx = 1
        for vertex in arr.vertices():
            vertex.set_data(idx)
            conn_graph.add_node(idx)
            vertex_dict[idx] = vertex
            idx += 1

        for edge in arr.halfedges():
            status = FaceData.FREE if is_walkable_edge(edge) else FaceData.INVALID
            edge.set_data(status)

        for vertex in arr.vertices():
            related_edges = [edge for edge in vertex.incident_halfedges() if edge.data() == FaceData.FREE]
            possible_vertices = list(set([edge.source() for edge in related_edges if edge.source().data() != vertex.data()] +
                                         [edge.target() for edge in related_edges if edge.target().data() != vertex.data()]))

            for target_vertex in possible_vertices:
                conn_graph.add_edge(vertex.data(), target_vertex.data())

        faces_status = [(face, face.is_unbounded() or not is_free_face(face)) for face in arr.faces()]

        for face, is_not_free in faces_status:
            if is_not_free:
                face.set_data(-1)
            else:
                face.set_data(idx)
                face_vertices = [edge.source().data() for edge in face.outer_ccb()]
                face_dict[idx] = face_vertices
                idx += 1

        return conn_graph, vertex_dict, face_dict

    def find_face_index(self, p, pl):
        """
        Get a point and find the index of the face in the arrangement that has it
        """
        p = TPoint(p.x(), p.y())
        obj = pl.locate(p)
        if type(obj) is Face:
            return obj.data()
        if type(obj) is Halfedge:
            return obj.face().data()
        return -1

    def find_valid_path(self, g_path, source, target, vertex_dict):
        """
        Convert a graph path to a valid motion planning path.
        We do that by connecting midpoints of edges we pass - and since the arrangement
        has circle curves, we might need split some edges in half (for exact motion).
        """
        path = []
        path.append(PathPoint(source))
        for point in g_path[1:-1]:
            vertex = vertex_dict[point]
            ker_point = self.to_ker_point_2(vertex.point())
            path.append(PathPoint(ker_point))
        path.append(PathPoint(target))
        return Path(path)
