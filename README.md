# Robot Planning and Decision Making

My solutions to the programming exercises of *Planning and Decision Making for Autonomous Robots*
(PDM4AR), ETH Zurich, autumn 2022. Solved individually in Python.

| Exercise | Topic | Code |
| --- | --- | --- |
| [01 Hello world](ex01-hello-world) | Warm-up: lexicographic comparison of two sequences | [`ex1.py`](ex01-hello-world/ex1.py) |
| [02 Graph search](ex02-graph-search) | Depth-first, breadth-first and iterative-deepening search | [`algo.py`](ex02-graph-search/algo.py) |
| [03 Informed graph search](ex03-informed-graph-search) | Uniform-cost search and A\* | [`algo.py`](ex03-informed-graph-search/algo.py) |
| [05 Dubins path](ex05-dubins-path) | Dubins and Reeds-Shepp paths for a car | [`algo.py`](ex05-dubins-path/algo.py) |
| [06 Collision checking](ex06-collision-checking) | Geometric collision primitives and a path collision checker | [`collision_primitives.py`](ex06-collision-checking/collision_primitives.py), [`collision_checker.py`](ex06-collision-checking/collision_checker.py) |
| [07 Optimization](ex07-optimization) | Graded: voyage planning as a mixed-integer linear program (PuLP) | [`ex07.py`](ex07-optimization/ex07.py) |
| [08 Driving games](ex08-driving-games) | Graded final: sampling-based path planner and path-following controller for a car | [`agent.py`](ex08-driving-games/agent.py) |

Exercise 04 (dynamic programming) was not completed.

## Running the code

The files plug into the course's exercise framework, which provides the problem setups, simulation and
visualization. They were written for its 2022 version,
[PDM4AR/exercises at `89e8039`](https://github.com/PDM4AR/exercises/tree/89e8039462ae80e2a3aefbb7010f062d897b2d80).
To run one, copy it into `src/pdm4ar/exercises/exNN/` there and follow the course's setup instructions.
The full problem statements are in that version's
[`docs/`](https://github.com/PDM4AR/exercises/tree/89e8039462ae80e2a3aefbb7010f062d897b2d80/docs) folder.

The code builds on the course's exercise templates, which are licensed under GPL-3.0 ([`LICENSE`](LICENSE)).
