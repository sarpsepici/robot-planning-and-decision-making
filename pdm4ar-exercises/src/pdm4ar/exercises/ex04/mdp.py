from abc import ABC, abstractmethod
from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from pdm4ar.exercises.ex04.structures import Action, Policy, State, ValueFunc


class GridMdp:
    def __init__(self, grid: NDArray[np.int], gamma: float = 0.9):
        assert len(grid.shape) == 2, "Map is invalid"
        self.grid = grid
        """The map"""
        self.gamma: float = gamma
        """Discount factor"""

    def get_transition_prob(self, state: State, action: Action, next_state: State) -> float:
        """Returns P(next_state | state, action)"""


        st = state
        next_st = next_state
        act = action
        val = self.grid[st]
        nx_val = self.grid[next_st]
        val_try = self.grid[[4,4]]
        tp_lf = self.grid[0,0]
        tp_lf_val = self.grid[[0,0]]
        bt_rg = self.grid[4,4]
        st_cell = self.grid[1,2]
        go_cell = self.grid[3,3]
        sw1 = self.grid[0,4]
        sw2 = self.grid[3,0]
        

        trans_prob = 0
        return trans_prob

    def stage_reward(self, state: State, action: Action) -> float:

        # todo



        pass


class GridMdpSolver(ABC):
    @staticmethod
    @abstractmethod
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        pass
