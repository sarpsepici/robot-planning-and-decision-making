from abc import ABC, abstractmethod
from dataclasses import dataclass
from osmnx.distance import great_circle_vec

from pdm4ar.exercises.ex02.structures import X, Path
from pdm4ar.exercises.ex03.structures import WeightedGraph, TravelSpeed


@dataclass
class InformedGraphSearch(ABC):
    graph: WeightedGraph

    @abstractmethod
    def path(self, start: X, goal: X) -> Path:
        # need to introduce weights!
        pass

@dataclass
class UniformCostSearch(InformedGraphSearch):
    def path(self, start: X, goal: X) -> Path:

        Q = {start: 0}
        cost2reach_d = {start: 0}
        parents = {start: start}
        path0 = []

        while len(Q)>0:

            Q0 = list(Q.keys())[0]

            if Q0 == goal:
                path0 = [goal]
                while parents[path0[0]] != start:
                    path0.insert(0, parents[path0[0]])
                if start != goal:
                    path0.insert(0, start)
                return path0

            succ = self.graph.adj_list[Q0]

            for i in succ:
                wgt = self.graph.get_weight(Q0, i)
                cost2reach = cost2reach_d[Q0] + wgt
                if i in list(cost2reach_d.keys()):
                    if cost2reach_d[i] > cost2reach:
                        cost2reach_d[i] = cost2reach
                        if i in list(Q.keys()):
                            Q[i] = cost2reach
                        parents[i] = Q0
                if i not in list(parents.keys()):
                    parents[i] = Q0
                    cost2reach_d[i] = cost2reach
                    Q[i] = cost2reach

            Q.pop(Q0)
            Q_tem = sorted(Q.items(), key=lambda x:x[1])
            Q = dict(Q_tem)

        return []        

@dataclass
class Astar(InformedGraphSearch):

    def heuristic(self, u: X, v: X) -> float:
        
        coord_U = self.graph.get_node_coordinates(u)
        coord_V = self.graph.get_node_coordinates(v)
        dist = great_circle_vec(coord_U[0], coord_U[1], coord_V[0], coord_V[1])
        # dist = abs(coord_U[0] - coord_V[0]) + abs(coord_U[1] - coord_V[1])
        speed = TravelSpeed.HIGHWAY.value
        travel_time = (dist / speed) 

        return travel_time
        
    def path(self, start: X, goal: X) -> Path:
        
        Q = {start: 0}
        cost2reach_d = {start: 0}
        parents = {start: start}
        path0 = []

        while len(Q)>0:

            Q0 = list(Q.keys())[0]

            if Q0 == goal:
                path0 = [goal]
                while parents[path0[0]] != start:
                    path0.insert(0, parents[path0[0]])
                if start != goal:
                    path0.insert(0, start)
                return path0

            succ = self.graph.adj_list[Q0]

            for i in succ:

                wgt = self.graph.get_weight(Q0, i)
                cost2reach = cost2reach_d[Q0] + wgt
                heur = self.heuristic(i, goal)

                if i in list(cost2reach_d.keys()):
                    if cost2reach_d[i] > cost2reach:
                        cost2reach_d[i] = cost2reach
                        if i in list(Q.keys()):
                            Q[i] = heur + cost2reach
                        parents[i] = Q0

                if i not in list(parents.keys()):
                    parents[i] = Q0
                    cost2reach_d[i] = cost2reach
                    Q[i] = heur + cost2reach

            Q.pop(Q0)
            Q_temp = sorted(Q.items(), key=lambda x:x[1])
            Q = dict(Q_temp)

        return []


def compute_path_cost(wG: WeightedGraph, path: Path):
    """A utility function to compute the cumulative cost along a path"""
    if not path:
        return float("inf")
    total: float = 0
    for i in range(1, len(path)):
        inc = wG.get_weight(path[i - 1], path[i])
        total += inc
    return total
