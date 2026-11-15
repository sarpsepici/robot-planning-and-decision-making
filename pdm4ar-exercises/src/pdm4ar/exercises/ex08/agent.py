import random
from dataclasses import dataclass
from typing import Sequence

from commonroad.scenario.lanelet import LaneletNetwork
from dg_commons import PlayerName
from dg_commons.sim.goals import PlanningGoal
from dg_commons.sim import SimObservations, InitSimObservations
from dg_commons.sim.agents import Agent
from dg_commons.sim.models.obstacles import StaticObstacle
from dg_commons.sim.models.vehicle import VehicleCommands
from dg_commons.sim.models.vehicle_structures import VehicleGeometry
from dg_commons.sim.models.vehicle_utils import VehicleParameters

from shapely.geometry import Point
from shapely.geometry import LineString
import numpy as np


@dataclass(frozen=True)
class Pdm4arAgentParams:
    param1: float = 0.2

def euclidian_dist(p1: Point, p2: Point) -> float:
    return np.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)

def nearest(Vertices, p: Point) -> Point:
    dist_vec = []
    for i in Vertices:
        dist_vec.append(euclidian_dist(i, p))
    return np.argmin(dist_vec)

counter_0 = 0 ##########################################
Path = []


class Pdm4arAgent(Agent):
    """This is the PDM4AR agent.
    Do *NOT* modify the naming of the existing methods and the input/output types.
    Feel free to add additional methods, objects and functions that help you to solve the task"""

    def __init__(self,
                 sg: VehicleGeometry,
                 sp: VehicleParameters
                 ):
        self.sg = sg
        self.sp = sp
        self.name: PlayerName = None
        self.goal: PlanningGoal = None
        self.lanelet_network: LaneletNetwork = None
        self.static_obstacles: Sequence[StaticObstacle] = None
        # feel free to remove/modify  the following
        self.params = Pdm4arAgentParams()

    def on_episode_init(self, init_obs: InitSimObservations):
        """This method is called by the simulator at the beginning of each episode."""
        self.name = init_obs.my_name
        self.goal = init_obs.goal
        self.lanelet_network = init_obs.dg_scenario.lanelet_network
        self.static_obstacles = list(init_obs.dg_scenario.static_obstacles.values())
        
        minx = 1000
        miny = 1000
        maxx = 0
        maxy = 0
        for i in range(len(self.lanelet_network.lanelet_polygons)):
            if self.lanelet_network.lanelet_polygons[i]._min[0] < minx:
                minx = self.lanelet_network.lanelet_polygons[i]._min[0]
            if self.lanelet_network.lanelet_polygons[i]._min[1] < miny:
                miny = self.lanelet_network.lanelet_polygons[i]._min[1]
            if self.lanelet_network.lanelet_polygons[i]._max[0] > maxx:
                maxx = self.lanelet_network.lanelet_polygons[i]._max[0]
            if self.lanelet_network.lanelet_polygons[i]._max[1] > maxy:
                maxy = self.lanelet_network.lanelet_polygons[i]._max[1]

        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy


        Vertices = [self.goal.goal.centroid]
        Parents = [self.goal.goal.centroid]
        Edges = []

        counter = 0
        while counter<200:

            point_x = self.minx + random.random() * (self.maxx - self.minx)
            point_y = self.miny + random.random() * (self.maxy - self.miny)
            point_np = np.array([point_x, point_y])
            p_shapely = Point(point_x, point_y)

            if not LaneletNetwork.find_lanelet_by_position(self.lanelet_network,[point_np])[0]:
                continue

            for obs in self.static_obstacles:
                booly = 0
                if p_shapely.intersects(obs.shape):
                    booly = 1
                    break
            if booly:
                continue

            p_nearest = Vertices[nearest(Vertices, p_shapely)]

            line = LineString([(p_shapely.x, p_shapely.y), (p_nearest.x, p_nearest.y)])

            for obs in self.static_obstacles:
                booly = 0
                if obs.shape.distance(line) < self.sg.width:
                    booly = 1
                    break
            if booly:
                continue

            if not p_nearest == self.goal.goal.centroid:
                p_nearest_parent = Parents[Vertices.index(p_nearest)]
                line_next = LineString([(p_nearest.x, p_nearest.y), (p_nearest_parent.x, p_nearest_parent.y)])

                L1_x = p_nearest.x - p_shapely.x
                L1_y = p_nearest.y - p_shapely.y
                L2_x = p_nearest_parent.x - p_nearest.x
                L2_y = p_nearest_parent.y - p_nearest.y

                theta = np.arccos((L1_x * L2_x + L1_y * L2_y) / (line.length * line_next.length))

                if theta > np.pi/4:
                    continue


            Vertices.append(p_shapely)
            Parents.append(p_nearest)
            Edges.append(line)

            counter += 1

        self.Vertices = Vertices
        self.Parents = Parents
        self.Edges = Edges
        
        print("*****************")

    def get_commands(self, sim_obs: SimObservations) -> VehicleCommands:
        """ This method is called by the simulator at each time step.
        For instance, this is how you can get your current state from the observations:
        my_current_state: VehicleState = sim_obs.players[self.name].state

        :param sim_obs:
        :return:
        """
        
        VehicleState = sim_obs.players[self.name].state
        p_curr = Point(VehicleState.x, VehicleState.y)

        global counter_0
        global Path


        if counter_0 == 0:

            Path = []
            theta = 1000
            Vertices_test = self.Vertices
            Parents_test = self.Parents
            p_nearest = self.Vertices[nearest(self.Vertices, p_curr)]

            if not p_nearest == self.goal.goal.centroid:
                p_nearest_parent = self.Parents[self.Vertices.index(p_nearest)]
                line = LineString([(p_curr.x, p_curr.y), (p_nearest.x, p_nearest.y)])
                line_next = LineString([(p_nearest.x, p_nearest.y), (p_nearest_parent.x, p_nearest_parent.y)])

                L1_x = p_nearest.x - p_curr.x
                L1_y = p_nearest.y - p_curr.y
                L2_x = p_nearest_parent.x - p_nearest.x
                L2_y = p_nearest_parent.y - p_nearest.y

                theta = np.arccos((L1_x * L2_x + L1_y * L2_y) / (line.length * line_next.length))


            while theta > np.pi/4:

                Vertices_test.remove(p_nearest)
                Parents_test.remove(p_nearest_parent)
                p_nearest = Vertices_test[nearest(Vertices_test, p_curr)]

                if not p_nearest == self.goal.goal.centroid:
                    p_nearest_parent = Parents_test[Vertices_test.index(p_nearest)]
                    line = LineString([(p_curr.x, p_curr.y), (p_nearest.x, p_nearest.y)])
                    line_next = LineString([(p_nearest.x, p_nearest.y), (p_nearest_parent.x, p_nearest_parent.y)])

                    L1_x = p_nearest.x - p_curr.x
                    L1_y = p_nearest.y - p_curr.y
                    L2_x = p_nearest_parent.x - p_nearest.x
                    L2_y = p_nearest_parent.y - p_nearest.y

                    theta = np.arccos((L1_x * L2_x + L1_y * L2_y) / (line.length * line_next.length))


            p_line_end = p_nearest
            Path.append(LineString([(p_curr.x, p_curr.y),(p_nearest.x, p_nearest.y)]))

            while not p_line_end == self.goal.goal.centroid:

                p_line_st = p_line_end
                p_line_end = self.Parents[self.Vertices.index(p_line_st)]

                Path.append(LineString([(p_line_st.x, p_line_st.y),(p_line_end.x, p_line_end.y)]))

            counter_0 += 1



        line_dist_list = []
        for i in Path:
            line_dist_list.append(p_curr.distance(i))
        
        line_curr = Path[np.argmin(line_dist_list)]
        dist_err = line_dist_list[np.argmin(line_dist_list)]

        angle_car = np.arctan2(np.sin(VehicleState.psi),np.cos(VehicleState.psi))
        angle_line = np.arctan2(line_curr.coords[1][1] - line_curr.coords[0][1], line_curr.coords[1][0] - line_curr.coords[0][0])
        angle_err = angle_car - angle_line

        left_turn_booly = -1 * np.sign(((line_curr.coords[1][0] - line_curr.coords[0][0])*(p_curr.y - line_curr.coords[0][1]) - (line_curr.coords[1][1] - line_curr.coords[0][1])*(p_curr.x - line_curr.coords[0][0])))

        #rnd_acc = random.random() * self.params.param1
        rnd_acc = 0.1 * (20 - VehicleState.vx)
        #rnd_ddelta = (random.random()-.5) * self.params.param1
        rnd_ddelta = 100 * left_turn_booly * (dist_err)

        return VehicleCommands(acc=rnd_acc, ddelta=rnd_ddelta)

