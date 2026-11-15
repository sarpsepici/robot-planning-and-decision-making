from typing import List
import numpy as np
from dg_commons import SE2Transform
from pdm4ar.exercises.ex06.collision_primitives import CollisionPrimitives
from pdm4ar.exercises_def.ex06.structures import (
    Polygon,
    GeoPrimitive,
    Point,
    Segment,
    Circle,
    Triangle,
    Path,
)

##############################################################################################
############################# This is a helper function. #####################################
# Feel free to use this function or not

COLLISION_PRIMITIVES = {
    Point: {
        Circle: lambda x, y: CollisionPrimitives.circle_point_collision(y, x),
        Triangle: lambda x, y: CollisionPrimitives.triangle_point_collision(y, x),
        Polygon: lambda x, y: CollisionPrimitives.polygon_point_collision(y, x),
    },
    Segment: {
        Circle: lambda x, y: CollisionPrimitives.circle_segment_collision(y, x),
        Triangle: lambda x, y: CollisionPrimitives.triangle_segment_collision(y, x),
        Polygon: lambda x, y: CollisionPrimitives.polygon_segment_collision_aabb(y, x),
    },
    Triangle: {
        Point: CollisionPrimitives.triangle_point_collision,
        Segment: CollisionPrimitives.triangle_segment_collision,
    },
    Circle: {
        Point: CollisionPrimitives.circle_point_collision,
        Segment: CollisionPrimitives.circle_segment_collision,
    },
    Polygon: {
        Point: CollisionPrimitives.polygon_point_collision,
        Segment: CollisionPrimitives.polygon_segment_collision_aabb,
    },
}


def check_collision(p_1: GeoPrimitive, p_2: GeoPrimitive) -> bool:
    """
    Checks collision between 2 geometric primitives
    Note that this function only uses the functions that you implemented in CollisionPrimitives class.
        Parameters:
                p_1 (GeoPrimitive): Geometric Primitive
                p_w (GeoPrimitive): Geometric Primitive
    """
    assert type(p_1) in COLLISION_PRIMITIVES, "Collision primitive does not exist."
    assert (
        type(p_2) in COLLISION_PRIMITIVES[type(p_1)]
    ), "Collision primitive does not exist."

    collision_func = COLLISION_PRIMITIVES[type(p_1)][type(p_2)]

    return collision_func(p_1, p_2)


##############################################################################################


class CollisionChecker:
    """
    This class implements the collision check ability of a simple planner for a circular differential drive robot.

    Note that check_collision could be used to check collision between given GeoPrimitives
    check_collision function uses the functions that you implemented in CollisionPrimitives class.
    """

    def __init__(self):
        pass

    def path_collision_check(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
        intersections = []
        segment_list = []
        len_t = len(t.waypoints)
        for i in range(len_t-1):
            segment_list.append(Segment(t.waypoints[i], t.waypoints[i+1]))

        len_segment_list = len(segment_list)
        len_obstacles = len(obstacles)

        for i in range(len_segment_list):

            theta = np.arctan2(segment_list[i].p2.y - segment_list[i].p1.y, segment_list[i].p2.x - segment_list[i].p1.x)

            upper_segment = Segment(Point(segment_list[i].p1.x - r * np.sin(theta), segment_list[i].p1.y + r * np.cos(theta)), Point(segment_list[i].p2.x - r * np.sin(theta), segment_list[i].p2.y + r * np.cos(theta)))

            upper_middle_segment = Segment(Point(segment_list[i].p1.x - r * np.sin(theta + np.pi/4), segment_list[i].p1.y + r * np.cos(theta + np.pi/4)), Point(segment_list[i].p2.x - r * np.sin(theta - np.pi/4), segment_list[i].p2.y + r * np.cos(theta - np.pi/4)))

            middle_segment = Segment(Point(segment_list[i].p1.x - r * np.cos(theta), segment_list[i].p1.y - r * np.sin(theta)), Point(segment_list[i].p2.x + r * np.cos(theta), segment_list[i].p2.y + r * np.sin(theta)))

            middle_lower_segment = Segment(Point(segment_list[i].p1.x + r * np.sin(theta - np.pi/4), segment_list[i].p1.y - r * np.cos(theta - np.pi/4)), Point(segment_list[i].p2.x + r * np.sin(theta - np.pi/4), segment_list[i].p2.y + r * np.cos(theta - np.pi/4)))

            lower_segment = Segment(Point(segment_list[i].p1.x + r * np.sin(theta), segment_list[i].p1.y - r * np.cos(theta)), Point(segment_list[i].p2.x + r * np.sin(theta), segment_list[i].p2.y - r * np.cos(theta)))

            booly = True
            for j in range(len_obstacles):
                if check_collision(upper_segment, obstacles[j]):
                    intersections.append(i)
                    booly = False
            
            if booly:
                for j in range(len_obstacles):
                    if check_collision(middle_segment, obstacles[j]):
                        intersections.append(i)
                        booly = False

                    sample_circle = []
                    phi = 0
                    for k in range(15):
                        phi = phi + k * 2 * np.pi / 15
                        sample_circle.append(Point(segment_list[i].p1.x - r * np.cos(phi), segment_list[i].p1.y - r * np.sin(phi)))
                        sample_circle.append(Point(segment_list[i].p2.x - r * np.cos(phi), segment_list[i].p2.y - r * np.sin(phi)))

                    len_sample_circle = len(sample_circle)
                    for f in range(len_sample_circle):
                        if check_collision(sample_circle[f], obstacles[j]):
                            booly = False

            if booly:
                for j in range(len_obstacles):
                    if check_collision(lower_segment, obstacles[j]):
                        intersections.append(i)

        return intersections

    def path_collision_check_occupancy_grid(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1

        In this method, you will generate an occupancy grid of the given map.
        Then, occupancy grid will be used to check collisions.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """

        intersections = []

        p_min_1, p_max_1 = t.get_boundaries()
        rad = r

        map_matrix = []
        map_x = p_max_1.x - p_min_1.x
        map_y = p_max_1.y - p_min_1.y
        print(map_x)
        print(map_y)

        len_obstacles = len(obstacles)
        for i in range(len_obstacles):

            if isinstance(obstacles[i], Circle):
                
                c_c = obstacles[i].center
                c_cx = c_c.x
                c_cy = c_c.y
                c_r = obstacles[i].radius

                for j in range(12):
                    alpha = j * np.pi / 6 
                    p_temp = np.array([c_cx + c_r * np.cos(alpha), c_cy + c_r * np.sin(alpha)])
                    ptemp = np.floor(p_temp / 0.5)
                    p_temp = p_temp.tolist()
                    if (p_temp not in map_matrix):
                        map_matrix.append(p_temp)



            # if isinstance(obstacles[i], Triangle):

            # if isinstance(obstacles[i], Polygon):


        segment_list = []
        len_path = len(t.waypoints)
        for i in range(len_path-1):
            segment_list.append([t.waypoints[i], t.waypoints[i+1]])

        for i in range(len_path-1):

            p1 = t.waypoints[i]
            p1x = p1.x
            p1y = p1.y
            p2 = t.waypoints[i+1]
            p2x = p2.x
            p2y = p2.y

            for j in range(11):
                
                p_temp = np.array([p1x + (i * (p2x-p1x) / 10), p1y + (i * (p2y-p1y) / 10)])
                p_temp = np.floor(p_temp / 0.5)
                p_temp = p_temp.tolist()

                if p_temp in map_matrix:
                    if i not in intersections:
                        intersections.append(i)

        return intersections

    def path_collision_check_r_tree(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1

        In this method, you will build an R-Tree of the given obstacles.
        You are free to implement your own R-Tree or you could use STRTree of shapely module.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
        return []

    def collision_check_robot_frame(
        self,
        r: float,
        current_pose: SE2Transform,
        next_pose: SE2Transform,
        observed_obstacles: List[GeoPrimitive],
    ) -> bool:
        """
        Returns there exists a collision or not during the movement of a circular differential drive robot until its next pose.

            Parameters:
                    r (float): Radius of circular differential drive robot
                    current_pose (SE2Transform): Current pose of the circular differential drive robot
                    next_pose (SE2Transform): Next pose of the circular differential drive robot
                    observed_obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives in robot frame
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
        return False

    def path_collision_check_safety_certificate(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1

        In this method, you will implement the safety certificates procedure for collision checking.
        You are free to use shapely to calculate distance between a point and a GoePrimitive.
        For more information, please check Algorithm 1 inside the following paper:
        https://journals.sagepub.com/doi/full/10.1177/0278364915625345.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
        return []
