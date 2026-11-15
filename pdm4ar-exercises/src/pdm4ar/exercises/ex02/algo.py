from abc import abstractmethod, ABC
from typing import Tuple

from pdm4ar.exercises.ex02.structures import AdjacencyList, X, Path, OpenedNodes


class GraphSearch(ABC):
    @abstractmethod
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        """
        :param graph: The given graph as an adjacency list
        :param start: The initial state (i.e. a node)
        :param goal: The goal state (i.e. a node)
        :return: The path from start to goal as a Sequence of states, None if a path does not exist
        """
        pass


class DepthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        
        Q = [start]
        openedNodes = []
        everyOpened = [start]
        parents = [start]
        path = [goal]
         
        while len(Q)>0:

            if Q[0] == goal:

                while parents[everyOpened.index(path[0])] != start:
                    path.insert(0, parents[everyOpened.index(path[0])])
                if goal != start:
                    path.insert(0, start)
                
                openedNodes.append(Q[0])

                return path, openedNodes
                
            succ = list(graph[Q[0]])
            succ.sort()
            succ.reverse()

            openedNodes.append(Q[0])

            for i in succ:
                if i not in everyOpened:
                    Q.insert(1, i)        

            succ.reverse()   

            for i in succ:
                if i not in everyOpened:
                    everyOpened.append(i)
                    parents.append(Q[0])
                        
            Q.remove(Q[0])
        
        return [], openedNodes

class BreadthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
       
        Q = [start]
        openedNodes = []
        everyOpened = [start]
        parents = [start]
        path = [goal]
         
        while len(Q)>0:

            if Q[0] == goal:

                while parents[everyOpened.index(path[0])] != start:
                    path.insert(0, parents[everyOpened.index(path[0])])
                if goal != start:
                    path.insert(0, start)
                
                openedNodes.append(Q[0])

                return path, openedNodes
                
            succ = list(graph[Q[0]])
            succ.sort()

            openedNodes.append(Q[0])

            for i in succ:
                if i not in everyOpened:
                    Q.append(i)       

            for i in succ:
                if i not in everyOpened:
                    everyOpened.append(i)
                    parents.append(Q[0])
                        
            Q.remove(Q[0])
          
        return [], openedNodes

class IterativeDeepening(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        
        d = 1
        booly = True
        Q = [start]
        everyOpened = [start]

        while booly:
            
            booly = False
            Q = [start]
            openedNodes = []
            everyOpened = [start]
            parents = [start]
            path = [goal]
            path_try = []
            
            while len(Q)>0:

                if Q[0] == goal:

                    while parents[everyOpened.index(path[0])] != start:
                        path.insert(0, parents[everyOpened.index(path[0])])
                    if goal != start:
                        path.insert(0, start)
                    
                    openedNodes.append(Q[0])

                    return path, openedNodes
                
                path_try. insert(0,Q[0])
                while parents[everyOpened.index(path_try[0])] != start:
                    path_try.insert(0, parents[everyOpened.index(path_try[0])])
                if start not in path_try:
                    path_try.insert(0,start)
                if len(path_try) >= d:
                    path_try = []
                    openedNodes.append(Q[0])
                    Q.remove(Q[0])
                    booly = True
                    continue

                path_try = []
                succ = list(graph[Q[0]])
                succ.sort()
                succ.reverse()

                openedNodes.append(Q[0])

                for i in succ:
                    if i not in everyOpened:
                        Q.insert(1, i)        

                succ.reverse()   

                for i in succ:
                    if i not in everyOpened:
                        everyOpened.append(i)
                        parents.append(Q[0])
                            
                Q.remove(Q[0])

            d = d+1
            
        return [], openedNodes