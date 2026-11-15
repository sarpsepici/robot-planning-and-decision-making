from pdm4ar.exercises_def.ex06.structures import *
import numpy as np
import triangle

class CollisionPrimitives:
    """
    Class of collusion primitives
    """

    @staticmethod
    def circle_point_collision(c: Circle, p: Point) -> bool:

        booly = False

        point_x = p.x
        point_y = p.y
        center_x = c.center.x
        center_y = c.center.y

        if (np.sqrt((center_x - point_x)**2 + (center_y - point_y)**2)) < c.radius:
            booly = True

        return booly

    @staticmethod
    def line_point_collision(segment: Segment, p: Point) -> bool:

        px = p.x
        py = p.y
        x1 = segment.p1.x
        y1 = segment.p1.y
        x2 = segment.p2.x
        y2 = segment.p2.y

        len_line = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        d1 = np.sqrt((x1-px)**2 + (y1-py)**2)
        d2 = np.sqrt((x2-px)**2 + (y2-py)**2)

        if np.abs((d1+d2)-len_line) < 0.001:
            return True

        return False

    @staticmethod
    def triangle_point_collision(t: Triangle, p: Point) -> bool:

        booly = False

        px = p.x
        py = p.y

        vert_1 = t.v1
        x1 = vert_1.x
        y1 = vert_1.y

        vert_2 = t.v2
        x2 = vert_2.x
        y2 = vert_2.y

        vert_3 = t.v3
        x3 = vert_3.x
        y3 = vert_3.y

        areaOrig = abs( (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1) )

        area1 = abs( (x1-px)*(y2-py) - (x2-px)*(y1-py) )
        area2 = abs( (x2-px)*(y3-py) - (x3-px)*(y2-py) )
        area3 = abs( (x3-px)*(y1-py) - (x1-px)*(y3-py) )

        if np.abs(area1 + area2 + area3 - areaOrig) < 0.0001:
            booly = True

        return booly

    @staticmethod
    def polygon_point_collision(poly: Polygon, p: Point) -> bool:

        booly = False

        vert = poly.vertices
        len_vert = len(vert)

        vert_array = [[vert[0].x, vert[0].y]]

        for i in range(len_vert-1):
            vert_array.append([vert[i+1].x, vert[i+1].y])

        t = triangle.triangulate({'vertices': vert_array})

        new_vert = t['vertices'].tolist()
        new_tri = t['triangles'].tolist()
        lng = len(new_tri)

        for i in range(lng):
            
            vert_ind = new_tri[i]
            vert1 = new_vert[vert_ind[0]]
            vert2 = new_vert[vert_ind[1]]
            vert3 = new_vert[vert_ind[2]]
            tri_temp = Triangle(Point(vert1[0], vert1[1]), Point(vert2[0], vert2[1]), Point(vert3[0], vert3[1]))

            if (CollisionPrimitives.triangle_point_collision(tri_temp, p)):
                booly = True

        return booly

    @staticmethod
    def circle_segment_collision(c: Circle, segment: Segment) -> bool:

        cx = c.center.x
        cy = c.center.y
        rad = c.radius

        x1 = segment.p1.x
        y1 = segment.p1.y
        x2 = segment.p2.x
        y2 = segment.p2.y

        if CollisionPrimitives.circle_point_collision(c, segment.p1):
            return True
        if CollisionPrimitives.circle_point_collision(c, segment.p2):
            return True

        len_line = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        dot = ( ((cx-x1)*(x2-x1)) + ((cy-y1)*(y2-y1)) ) / pow(len_line,2)
        closest_x = x1 + (dot * (x2-x1))
        closest_y = y1 + (dot * (y2-y1))

        if not CollisionPrimitives.line_point_collision(segment, Point(closest_x, closest_y)):
            return False

        distance = np.sqrt((closest_x - cx)**2 + (closest_y - cy)**2)

        if distance < rad:
            return True

        return False

    @staticmethod
    def triangle_segment_collision(t: Triangle, segment: Segment) -> bool:

        x1 = segment.p1.x
        y1 = segment.p1.y
        x2 = segment.p2.x
        y2 = segment.p2.y

        for i in range(16):

            point_temp = Point(x1 + (i * (x2-x1) / 15), y1 + (i * (y2-y1) / 15))

            if CollisionPrimitives.triangle_point_collision(t, point_temp):
                return True

        return False

    @staticmethod
    def polygon_segment_collision(p: Polygon, segment: Segment) -> bool:

        x1 = segment.p1.x
        y1 = segment.p1.y
        x2 = segment.p2.x
        y2 = segment.p2.y

        for i in range(16):

            point_temp = Point(x1 + (i * (x2-x1) / 15), y1 + (i * (y2-y1) / 15))

            if CollisionPrimitives.polygon_point_collision(p, point_temp):
                return True

        return False

    @staticmethod
    def line_line_collision(segment1: Segment, segment2: Segment) -> bool:

        x1_s1 = segment1.p1.x
        y1_s1 = segment1.p1.y
        x2_s1 = segment1.p2.x
        y2_s1 = segment1.p2.y

        x1_s2 = segment2.p1.x
        y1_s2 = segment2.p1.y
        x2_s2 = segment2.p2.x
        y2_s2 = segment2.p2.y

        x_diff = [x1_s1 - x2_s1, x1_s2 - x2_s2]
        y_diff = [y1_s1 - y2_s1, y1_s2 - y2_s2]

        div = x_diff[0] * y_diff[1] - x_diff[1] * y_diff[0]

        if div == 0 :
            return False

        return True

    @staticmethod
    def polygon_segment_collision_aabb(p: Polygon, segment: Segment) -> bool:

        AABB_temp = CollisionPrimitives._poly_to_aabb(p)

        dx = AABB_temp.p_max.x - AABB_temp.p_min.x
        dy = AABB_temp.p_max.y - AABB_temp.p_min.y

        coord_00 = AABB_temp.p_min
        coord_10 = AABB_temp.p_min
        coord_10 = Point(coord_10.x + dx, coord_10.y)
        coord_01 = AABB_temp.p_min
        coord_01 = Point(coord_01.x, coord_01.y + dy)
        coord_11 = AABB_temp.p_max

        segment_1 = Segment(coord_00, coord_10)
        segment_2 = Segment(coord_10, coord_11)
        segment_3 = Segment(coord_11, coord_01)
        segment_4 = Segment(coord_01, coord_00)

        if ((coord_00.x<segment.p1.x) and (coord_11.x>segment.p1.x) and (coord_00.y<segment.p1.y) and (coord_11.y>segment.p1.y) and (coord_00.x<segment.p2.x) and (coord_11.x>segment.p2.x) and (coord_00.x<segment.p2.y) and (coord_11.x>segment.p2.y)):
            return True

        if CollisionPrimitives.line_line_collision(segment, segment_1):
            if CollisionPrimitives.polygon_segment_collision(p, segment):
                return True
        if CollisionPrimitives.line_line_collision(segment, segment_2):
            if CollisionPrimitives.polygon_segment_collision(p, segment):
                return True
        if CollisionPrimitives.line_line_collision(segment, segment_3):
            if CollisionPrimitives.polygon_segment_collision(p, segment):
                return True
        if CollisionPrimitives.line_line_collision(segment, segment_4):
            if CollisionPrimitives.polygon_segment_collision(p, segment):
                return True

        return False

    @staticmethod
    def _poly_to_aabb(g: Polygon) -> AABB:
        # todo feel free to implement functions that upper-bound a shape with an
        #  AABB or simpler shapes for faster collision checks

        p_min_1, p_max_1 = g.get_boundaries()

        return AABB(p_min=p_min_1, p_max=p_max_1)
