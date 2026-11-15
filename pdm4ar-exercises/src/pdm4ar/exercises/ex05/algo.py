from typing import Sequence

from dg_commons import SE2Transform

from pdm4ar.exercises.ex05.structures import *
from pdm4ar.exercises_def.ex05.utils import extract_path_points

 
class PathPlanner(ABC):
    @abstractmethod
    def compute_path(self, start: SE2Transform, end: SE2Transform) -> Sequence[SE2Transform]:
        pass


class Dubins(PathPlanner):
    def __init__(self, params: DubinsParam):
        self.params = params

    def compute_path(self, start: SE2Transform, end: SE2Transform) -> List[SE2Transform]:
        """ Generates an optimal Dubins path between start and end configuration

        :param start: the start configuration of the car (x,y,theta)
        :param end: the end configuration of the car (x,y,theta)

        :return: a List[SE2Transform] of configurations in the optimal path the car needs to follow
        """
        path = calculate_dubins_path(start_config=start, end_config=end, radius=self.params.min_radius)
        se2_list = extract_path_points(path)
        return se2_list


class ReedsShepp(PathPlanner):
    def __init__(self, params: DubinsParam):
        self.params = params

    def compute_path(self, start: SE2Transform, end: SE2Transform) -> Sequence[SE2Transform]:
        """ Generates a Reeds-Shepp *inspired* optimal path between start and end configuration

        :param start: the start configuration of the car (x,y,theta)
        :param end: the end configuration of the car (x,y,theta)

        :return: a List[SE2Transform] of configurations in the optimal path the car needs to follow 
        """
        path = calculate_reeds_shepp_path(start_config=start, end_config=end, radius=self.params.min_radius)
        se2_list = extract_path_points(path)
        return se2_list


#########################################################



def calculate_car_turning_radius(wheel_base: float, max_steering_angle: float) -> DubinsParam:

    return DubinsParam(min_radius = wheel_base / np.tan(max_steering_angle))


#########################################################


def calculate_turning_circles(current_config: SE2Transform, radius: float) -> TurningCircle:

    # dummy_circle = Curve.create_circle(center=SE2Transform.identity(), config_on_circle=SE2Transform.identity(),
    #                                    radius=0.1, curve_type=DubinsSegmentType.LEFT)
    
    pos = current_config.p
    x_pos = pos[0]
    y_pos = pos[1]
    tht = current_config.theta

    center_right = SE2Transform([x_pos + radius * np.sin(tht), y_pos - radius * np.cos(tht)], 0)
    center_left = SE2Transform([x_pos - radius * np.sin(tht), y_pos + radius * np.cos(tht)], 0)

    circle_right = Curve.create_circle(center=center_right, config_on_circle=SE2Transform.identity(),
                                       radius=radius, curve_type=DubinsSegmentType.RIGHT)
                                       
    circle_left = Curve.create_circle(center=center_left, config_on_circle=SE2Transform.identity(),
                                       radius=radius, curve_type=DubinsSegmentType.LEFT)

    return TurningCircle(left=circle_left, right=circle_right)


#########################################################


def calculate_arc_angle(ang_st: float, ang_end:float, type1:DubinsSegmentType) -> float:

    if ang_st >= ang_end:
        arc_ang = ang_st - ang_end
    else:
        arc_ang = 2 * np.pi - np.abs(ang_st - ang_end)

    if type1==DubinsSegmentType.RIGHT: 
        return arc_ang
    else:
        return 2 * np.pi - arc_ang


#########################################################



def calculate_tangent_btw_circles(circle_start: Curve, circle_end: Curve) -> List[Line]:
    # TODO implement here your solution

    center_start = circle_start.center
    center_end = circle_end.center

    rad = circle_start.radius

    pos_start = center_start.p
    x_start = pos_start[0]
    y_start = pos_start[1]

    pos_end = center_end.p
    x_end = pos_end[0]
    y_end = pos_end[1]

    type_start = circle_start.type
    type_end = circle_end.type

    theta_circle = np.arctan2((y_end - y_start), (x_end - x_start))

    if center_start == center_end:
        return []

    if type_start == DubinsSegmentType.LEFT and type_end == DubinsSegmentType.LEFT : 

        point_start = SE2Transform([x_start + np.cos(np.pi / 2 - theta_circle) * rad, y_start - np.sin(np.pi / 2 - theta_circle) * rad], theta_circle)
        point_end = SE2Transform([x_end + np.cos(np.pi / 2 - theta_circle) * rad, y_end - np.sin(np.pi / 2 - theta_circle) * rad], theta_circle)

        return [Line(start_config=point_start, end_config=point_end)]

    if type_start == DubinsSegmentType.RIGHT and type_end == DubinsSegmentType.RIGHT : 

        point_start = SE2Transform([x_start - np.cos(np.pi/2 - theta_circle) * rad, y_start + np.sin(np.pi/2 - theta_circle) * rad], theta_circle)
        point_end = SE2Transform([x_end - np.cos(np.pi/2 - theta_circle) * rad, y_end + np.sin(np.pi/2 - theta_circle) * rad], theta_circle)

        return [Line(start_config=point_start, end_config=point_end)]


    if (np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)) < (2*rad):
        return []
    
    phi_tangent = np.abs(np.arccos((2 * rad) / (np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2))))

    if type_start == DubinsSegmentType.LEFT and type_end == DubinsSegmentType.RIGHT : 

        if np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2) > 2 * rad:

            point_start = SE2Transform([x_start + np.cos(phi_tangent - theta_circle) * rad, y_start - np.sin(phi_tangent - theta_circle) * rad], np.pi / 2 - phi_tangent + theta_circle)
            point_end = SE2Transform([x_end - np.cos(phi_tangent - theta_circle) * rad, y_end + np.sin(phi_tangent - theta_circle) * rad], np.pi / 2 - phi_tangent + theta_circle)

            return [Line(start_config=point_start, end_config=point_end)]

        elif np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2) == 2 * rad:

            point = SE2Transform([x_start + np.cos(phi_tangent - theta_circle) * rad, y_start - np.sin(phi_tangent - theta_circle) * rad], np.pi / 2 + theta_circle)
            return [Line(start_config=point, end_config=point)]

        else :
            return []

    if type_start == DubinsSegmentType.RIGHT and type_end == DubinsSegmentType.LEFT : 

        if np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2) > 2 * rad:

            point_start = SE2Transform([x_start + np.cos(phi_tangent + theta_circle) * rad, y_start + np.sin(phi_tangent + theta_circle) * rad], phi_tangent + theta_circle - np.pi / 2)
            point_end = SE2Transform([x_end - np.cos(phi_tangent + theta_circle) * rad, y_end - np.sin(phi_tangent + theta_circle) * rad], phi_tangent + theta_circle - np.pi / 2)

            return [Line(start_config=point_start, end_config=point_end)]

        elif np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2) == 2 * rad:

            point = SE2Transform([x_start + np.cos(phi_tangent + theta_circle) * rad, y_start + np.sin(phi_tangent + theta_circle) * rad], theta_circle - np.pi / 2)
            return [Line(start_config=point, end_config=point)]

        else :
            return []


    return []  # i.e., [Line(),...]


#########################################################



def calculate_tangent_circle_btw_circles(circle_start: Curve, circle_end: Curve) -> List[Curve]:

    center_start = circle_start.center
    center_end = circle_end.center

    if center_start == center_end:
        return []

    if circle_start.type != circle_end.type:
        return[]

    rad = circle_start.radius

    pos_start = center_start.p
    x_start = pos_start[0]
    y_start = pos_start[1]

    pos_end = center_end.p
    x_end = pos_end[0]
    y_end = pos_end[1]

    base = (np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2))

    if base > 4 * rad :
        return []

    theta_circle = np.arctan2((y_end - y_start), (x_end - x_start))
    phi = np.abs(np.arccos(base / (4 * rad)))

    center_circle_up = SE2Transform([x_start + 2 * rad * np.cos(theta_circle + phi), y_start + 2 * rad * np.sin(theta_circle + phi)], 0)
    center_circle_down = SE2Transform([x_start + 2 * rad * np.cos(phi - theta_circle), y_start - 2 * rad * np.sin(phi - theta_circle)], 0)


    if circle_start.type == DubinsSegmentType.RIGHT and circle_end.type == DubinsSegmentType.RIGHT:

        # start_configuration_up = SE2Transform([x_start + rad * np.cos(theta_circle + phi), y_start + rad * np.sin(theta_circle + phi)], phi + theta_circle - np.pi / 2)
        # end_configuration_up = SE2Transform([x_end - np.cos(phi - theta_circle) * rad, y_end + np.sin(phi - theta_circle) * rad], np.pi / 2 - phi + theta_circle)
        # Curve_up = Curve()

        start_configuration_down = SE2Transform([x_start + rad * np.cos(phi - theta_circle), y_start - rad * np.sin(phi - theta_circle)], (-1) *  (np.pi / 2 + phi - theta_circle))
        end_configuration_down = SE2Transform([x_end - np.cos(phi + theta_circle) * rad, y_end - np.sin(phi + theta_circle) * rad], np.pi / 2 + phi + theta_circle)

        ang_st = start_configuration_down.theta
        ang_end = end_configuration_down.theta

        arc_ang = calculate_arc_angle(ang_st, ang_end, DubinsSegmentType.LEFT)

        Curve_down = Curve(center=center_circle_down, start_config=start_configuration_down, end_config=end_configuration_down, radius=rad, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang)

        return [Curve_down]

    if circle_start.type == DubinsSegmentType.LEFT and circle_end.type == DubinsSegmentType.LEFT:

        start_configuration_up = SE2Transform([x_start + rad * np.cos(theta_circle + phi), y_start + rad * np.sin(theta_circle + phi)], np.pi / 2 + phi + theta_circle)
        end_configuration_up = SE2Transform([x_end - np.cos(phi - theta_circle) * rad, y_end + np.sin(phi - theta_circle) * rad], (-1) * (np.pi / 2 + phi - theta_circle))

        ang_st = start_configuration_up.theta
        ang_end = end_configuration_up.theta

        arc_ang = calculate_arc_angle(ang_st, ang_end, DubinsSegmentType.RIGHT)

        Curve_up = Curve(center=center_circle_up, start_config=start_configuration_up, end_config=end_configuration_up, radius=rad, curve_type=DubinsSegmentType.RIGHT, arc_angle=arc_ang)

        # start_configuration_down = SE2Transform([x_start + rad * np.cos(phi - theta_circle), y_start - rad * np.sin(phi - theta_circle)], theta_circle - phi - np.pi / 2)
        # end_configuration_down = SE2Transform([x_end - np.cos(phi + theta_circle) * rad, y_end - np.sin(phi + theta_circle) * rad], phi + theta_circle - np.pi / 2)
        # Curve_down = Curve()

        return [Curve_up]

    return []


#########################################################



def calculate_dubins_path(start_config: SE2Transform, end_config: SE2Transform, radius: float) -> Path:
    # TODO implement here your solution
    # Please keep segments with zero length in the return list & return a valid dubins path!

    if start_config == end_config:
        return [Line(SE2Transform.identity(), SE2Transform.identity()), Line(SE2Transform.identity(), SE2Transform.identity()), Line(SE2Transform.identity(), SE2Transform.identity())]

    circles_start = calculate_turning_circles(start_config, radius)
    circle_start_R = circles_start.right
    circle_start_L = circles_start.left

    circles_end = calculate_turning_circles(end_config, radius)
    circle_end_R = circles_end.right
    circle_end_L = circles_end.left


    # RSR
    
    tan_RSR_list = calculate_tangent_btw_circles(circle_start_R, circle_end_R)

    if tan_RSR_list:

        tan_RSR = tan_RSR_list[0]

        tan_RSR_st = tan_RSR.start_config
        tan_RSR_end = tan_RSR.end_config

        arc_ang_RSR_1 = calculate_arc_angle(start_config.theta, tan_RSR_st.theta, DubinsSegmentType.RIGHT)
        arc_ang_RSR_2 = calculate_arc_angle(tan_RSR_end.theta, end_config.theta, DubinsSegmentType.RIGHT)

        path1_RSR = Curve(center=circle_start_R.center, start_config=start_config, end_config=tan_RSR_st, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=arc_ang_RSR_1)
        path2_RSR = tan_RSR
        path3_RSR = Curve(center=circle_end_R.center, start_config=tan_RSR_end, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=arc_ang_RSR_2)

        len_RSR = path1_RSR.length + path2_RSR.length + path3_RSR.length
        path_RSR = [path1_RSR, path2_RSR, path3_RSR]

    else:
        arc_ang_RSR = calculate_arc_angle(start_config.theta, end_config.theta, DubinsSegmentType.RIGHT)
        return [Curve(center=circle_start_R.center, start_config=start_config, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=arc_ang_RSR), List(start_config=end_config, end_config=end_config), Curve(center=circle_start_R.center, start_config=end_config, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=0)]

    # LSL

    tan_LSL_list = calculate_tangent_btw_circles(circle_start_L, circle_end_L)

    if tan_LSL_list:

        tan_LSL = tan_LSL_list[0]

        tan_LSL_st = tan_LSL.start_config
        tan_LSL_end = tan_LSL.end_config

        arc_ang_LSL_1 = calculate_arc_angle(start_config.theta, tan_LSL_st.theta, DubinsSegmentType.LEFT)
        arc_ang_LSL_2 = calculate_arc_angle(tan_LSL_end.theta, end_config.theta, DubinsSegmentType.LEFT)

        path1_LSL = Curve(center=circle_start_L.center, start_config=start_config, end_config=tan_LSL_st, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang_LSL_1)
        path2_LSL = tan_LSL
        path3_LSL = Curve(center=circle_end_L.center, start_config=tan_LSL_end, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang_LSL_2)

        len_LSL = path1_LSL.length + path2_LSL.length + path3_LSL.length
        path_LSL = [path1_LSL, path2_LSL, path3_LSL]

    else:
        arc_ang_LSL = calculate_arc_angle(start_config.theta, end_config.theta, DubinsSegmentType.LEFT)
        return [Curve(center=circle_start_L.center, start_config=start_config, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang_LSL), List(start_config=end_config, end_config=end_config), Curve(center=circle_start_R.center, start_config=end_config, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=0)]

    # RSL

    tan_RSL_list = calculate_tangent_btw_circles(circle_start_R, circle_end_L)

    if tan_RSL_list:

        tan_RSL = tan_RSL_list[0]

        tan_RSL_st = tan_RSL.start_config
        tan_RSL_end = tan_RSL.end_config

        arc_ang_RSL_1 = calculate_arc_angle(start_config.theta, tan_RSL_st.theta, DubinsSegmentType.RIGHT)
        arc_ang_RSL_2 = calculate_arc_angle(tan_RSL_end.theta, end_config.theta, DubinsSegmentType.LEFT)

        path1_RSL = Curve(center=circle_start_R.center, start_config=start_config, end_config=tan_RSL_st, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=arc_ang_RSL_1)
        path2_RSL = tan_RSL
        path3_RSL = Curve(center=circle_end_L.center, start_config=tan_RSL_end, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang_RSL_2)

        len_RSL = path1_RSL.length + path2_RSL.length + path3_RSL.length
        path_RSL = [path1_RSL, path2_RSL, path3_RSL]

    else:
        len_RSL = 10000
        path_RSL = [Curve(center=circle_start_L.center, start_config=SE2Transform.identity(), end_config=SE2Transform.identity(), radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=0), Line(SE2Transform.identity(), SE2Transform.identity()), Curve(center=circle_start_L.center, start_config=SE2Transform.identity(), end_config=SE2Transform.identity(), radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=0)]


    # LSR

    tan_LSR_list = calculate_tangent_btw_circles(circle_start_L, circle_end_R)

    if tan_LSR_list:
        tan_LSR = tan_LSR_list[0]

        tan_LSR_st = tan_LSR.start_config
        tan_LSR_end = tan_LSR.end_config

        arc_ang_LSR_1 = calculate_arc_angle(start_config.theta, tan_LSR_st.theta, DubinsSegmentType.LEFT)
        arc_ang_LSR_2 = calculate_arc_angle(tan_LSR_end.theta, end_config.theta, DubinsSegmentType.RIGHT)

        path1_LSR = Curve(center=circle_start_L.center, start_config=start_config, end_config=tan_LSR_st, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang_LSR_1)
        path2_LSR = tan_LSR
        path3_LSR = Curve(center=circle_end_R.center, start_config=tan_LSR_end, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=arc_ang_LSR_2)

        len_LSR = path1_LSR.length + path2_LSR.length + path3_LSR.length
        path_LSR = [path1_LSR, path2_LSR, path3_LSR]

    else:
        len_LSR = 10000
        path_LSR = [Curve(center=circle_start_L.center, start_config=SE2Transform.identity(), end_config=SE2Transform.identity(), radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=0), Line(SE2Transform.identity(), SE2Transform.identity()), Curve(center=circle_start_L.center, start_config=SE2Transform.identity(), end_config=SE2Transform.identity(), radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=0)]

    # RLR

    tan_circ_RLR_list = calculate_tangent_circle_btw_circles(circle_start=circle_start_R, circle_end=circle_end_R)

    if tan_circ_RLR_list:

        tan_circ_RLR = tan_circ_RLR_list[0]

        tan_circ_RLR_st = tan_circ_RLR.start_config
        tan_circ_RLR_end = tan_circ_RLR.end_config

        arc_ang_RLR_1 = calculate_arc_angle(start_config.theta, tan_circ_RLR_st.theta, DubinsSegmentType.RIGHT)
        arc_ang_RLR_2 = calculate_arc_angle(tan_circ_RLR_end.theta, end_config.theta, DubinsSegmentType.RIGHT)

        path1_RLR = Curve(center=circle_start_R.center, start_config=start_config, end_config=tan_circ_RLR_st, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=arc_ang_RLR_1)
        path2_RLR = tan_circ_RLR
        path3_RLR = Curve(center=circle_end_R.center, start_config=tan_circ_RLR_end, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang_RLR_2)

        len_RLR = path1_RLR.length + path2_RLR.length + path3_RLR.length
        path_RLR = [path1_RLR, path2_RLR, path3_RLR]

    else:
        len_RLR = 10000
        path_RLR = [Line(SE2Transform.identity(), SE2Transform.identity()), Line(SE2Transform.identity(), SE2Transform.identity()), Line(SE2Transform.identity(), SE2Transform.identity())]

    # LRL
                
    tan_circ_LRL_list = calculate_tangent_circle_btw_circles(circle_start=circle_start_L, circle_end=circle_end_L)

    if tan_circ_LRL_list:

        tan_circ_LRL = tan_circ_LRL_list[0]

        tan_circ_LRL_st = tan_circ_LRL.start_config
        tan_circ_LRL_end = tan_circ_LRL.end_config

        arc_ang_LRL_1 = calculate_arc_angle(start_config.theta, tan_circ_LRL_st.theta, DubinsSegmentType.LEFT)
        arc_ang_LRL_2 = calculate_arc_angle(tan_circ_LRL_end.theta, end_config.theta, DubinsSegmentType.LEFT)

        path1_LRL = Curve(center=circle_start_L.center, start_config=start_config, end_config=tan_circ_LRL_st, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang_LRL_1)
        path2_LRL = tan_circ_LRL
        path3_LRL = Curve(center=circle_end_L.center, start_config=tan_circ_LRL_end, end_config=end_config, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=arc_ang_LRL_2)

        len_LRL = path1_LRL.length + path2_LRL.length + path3_LRL.length
        path_LRL = [path1_LRL, path2_LRL, path3_LRL]

    else:
        len_LRL = 10000
        path_LRL = [Line(SE2Transform.identity(), SE2Transform.identity()), Line(SE2Transform.identity(), SE2Transform.identity()), Line(SE2Transform.identity(), SE2Transform.identity())]


    length_vec = [len_RSR, len_LSL, len_RSL, len_LSR, len_RLR, len_LRL]
    path_vec = [path_RSR, path_LSL, path_RSL, path_LSR, path_RLR, path_LRL]

    ind = length_vec.index(min(length_vec))

    return path_vec[ind]  # e.g., [Curve(), Line(),..]




def calculate_reeds_shepp_path(start_config: SE2Transform, end_config: SE2Transform, radius: float) -> Path:
    # TODO implement here your solution
    # Please keep segments with zero length in the return list & return a valid dubins/reeds path!

    path_dub = calculate_dubins_path(start_config, end_config, radius)

    path_dub_1 = path_dub[0]
    path_dub_2 = path_dub[1]
    path_dub_3 = path_dub[2]

    path_reeds = calculate_dubins_path(end_config, start_config, radius)

    path1 = path_reeds[0]
    path2 = path_reeds[1]
    path3 = path_reeds[2]

    path1.gear = Gear.REVERSE
    st_cf_1 = path1.start_config
    end_cf_1 = path1.end_config
    path1.start_config = end_cf_1
    path1.end_config = st_cf_1


    path2.gear = Gear.REVERSE
    st_cf_2 = path2.start_config
    end_cf_2 = path2.end_config
    path2.start_config = end_cf_2
    path2.end_config = st_cf_2

    path3.gear = Gear.REVERSE
    st_cf_3 = path3.start_config
    end_cf_3 = path3.end_config
    path3.start_config = end_cf_3
    path3.end_config = st_cf_3

    path_reeds = [path3, path2, path1]

    len_path_dub = path_dub_1.length + path_dub_2.length + path_dub_3.length
    len_path_reeds = path1.length + path2.length + path3.length

    if len_path_dub < len_path_reeds:
        return path_dub

    return path_reeds # e.g., [Curve(..,gear=Gear.REVERSE), Curve(),..]
