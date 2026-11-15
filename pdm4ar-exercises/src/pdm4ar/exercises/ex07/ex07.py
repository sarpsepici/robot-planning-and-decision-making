import numpy as np
from pulp import *
from pdm4ar.exercises_def.ex07.structures import (
    ProblemVoyage,
    OptimizationCost,
    Island,
    Constraints,
    Feasibility,
    SolutionVoyage,
)


def solve_optimization(problem: ProblemVoyage) -> SolutionVoyage:
    """
    Solve the optimization problem enforcing the requested constraints.

    Parameters
    ---
    problem : ProblemVoyage
        Contains the problem data: cost to optimize, starting crew, tuple of islands,
        and information about the requested constraint (the constraints not set to `None` +
        the `voyage_order` constraint)

    Returns
    ---
    out : SolutionVoyage
        Contains the feasibility status of the problem, and the optimal voyage plan
        as a list of ints if problem is feasible, else `None`.
    """

    # toy examples with random voyage plans
    np.random.seed(None)
    if np.random.random() > 0.3:
        feasibility = Feasibility.feasible
        voyage_plan = list(np.random.randint(0, len(problem.islands), size=(min(7, len(problem.islands)),)))
    else:
        feasibility = Feasibility.unfeasible
        voyage_plan = None


    optimization_cost = problem.optimization_cost
    start_crew = problem.start_crew
    islands = problem.islands
    min_nights_individual_island = problem.constraints.min_nights_individual_island
    min_total_crew = problem.constraints.min_total_crew
    max_total_crew = problem.constraints.max_total_crew
    max_duration_individual_journey = problem.constraints.max_duration_individual_journey
    max_L1_distance_individual_journey = problem.constraints.max_L1_distance_individual_journey

    nb_islands = len(islands)
    nb_arch = islands[-1].arch - 1
    nb_isl_arch = (len(islands) - 2) // nb_arch

    prob = LpProblem("MILP", LpMinimize)

    x_list = [w+1 for w in range(nb_islands - 2)]
    x_list_L1_abs = [i for i in range(nb_arch-1)]
    x_list_L1_opt = [i for i in range(nb_arch+1)]

    x_vars = LpVariable.dicts("Islands", x_list, lowBound=0, upBound=1, cat=LpInteger)
    x_vars_L1_abs = LpVariable.dicts("Absolute", x_list_L1_abs, lowBound=0)
    x_vars_L1_opt = LpVariable.dicts("Optimization", x_list_L1_opt, lowBound=0)



    def l1_dist(is1: Island, is2: Island) -> float:
        return np.abs(is2.y - is1.y) + np.abs(is2.x - is1.x)



    # OPTIMIZATION



    if optimization_cost == OptimizationCost.min_total_nights:
        prob += lpSum([islands[a+1].nights * x_vars[a+1] for a in range(nb_islands - 2)])

    if optimization_cost == OptimizationCost.max_final_crew:
        prob += lpSum([(-1) * islands[a+1].delta_crew * x_vars[a+1] for a in range(nb_islands - 2)])
        

    
    if optimization_cost == OptimizationCost.min_total_travelled_L1_distance:

        l1_dist_st = 0
        for i in range(nb_isl_arch):
            l1_dist_st += (np.abs(islands[i+1].y - islands[0].y) + np.abs(islands[i+1].x - islands[0].x)) * x_vars[i+1]
        prob += l1_dist_st <= x_vars_L1_opt[0]

        l1_dist_end = 0
        for i in range(nb_isl_arch):
            l1_dist_end += (np.abs(islands[1 + (nb_arch-1) * nb_isl_arch + i].y - islands[nb_islands - 1].y) + np.abs(islands[1 + (nb_arch-1) * nb_isl_arch + i].x - islands[nb_islands - 1].x)) * x_vars[1 + (nb_arch-1) * nb_isl_arch + i]
        prob += l1_dist_end <= x_vars_L1_opt[1]


        for i in range(nb_arch-1):

            l1_x1 = 0
            l1_y1 = 0
            l1_x2 = 0
            l1_y2 = 0
            l1_dist_1 = 0
            l1_dist_2 = 0
            l1_dist_3 = 0
            l1_dist_4 = 0

            for j in range(nb_isl_arch):

                l1_x1 += islands[1 + i * nb_isl_arch + j].x * x_vars[1 + i * nb_isl_arch + j]
                l1_y1 += islands[1 + i * nb_isl_arch + j].y * x_vars[1 + i * nb_isl_arch + j]
                l1_x2 += islands[1 + (i+1) * nb_isl_arch + j].x * x_vars[1 + (i+1) * nb_isl_arch + j]
                l1_y2 += islands[1 + (i+1) * nb_isl_arch + j].y * x_vars[1 + (i+1) * nb_isl_arch + j]


            l1_dist_1 = (l1_y2 - l1_y1) + (l1_x2 - l1_x1)
            l1_dist_2 = (l1_y2 - l1_y1) + (l1_x1 - l1_x2)
            l1_dist_3 = (l1_y1 - l1_y2) + (l1_x2 - l1_x1)
            l1_dist_4 = (l1_y1 - l1_y2) + (l1_x1 - l1_x2)

            prob += l1_dist_1 <= x_vars_L1_opt[i+2]
            prob += l1_dist_2 <= x_vars_L1_opt[i+2]
            prob += l1_dist_3 <= x_vars_L1_opt[i+2]
            prob += l1_dist_4 <= x_vars_L1_opt[i+2]

        prob += lpSum(x_vars_L1_opt)


    if optimization_cost == OptimizationCost.min_total_sailing_time:

        x_list_min_sail = [i for i in range(nb_arch+1)]
        x_vars_min_sail = LpVariable.dicts("Min_sail", x_list_min_sail, lowBound=0)
        

        couple_max_duration_st = []
        couple_max_duration_mid = []
        couple_max_duration_end = []
        
        for i in range(nb_isl_arch):
            couple_max_duration_st.append(islands[i+1].arrival * x_vars[i+1] - islands[0].departure * x_vars[i+1])
        
        for i in range(nb_arch-1):
            departure = 0
            arrival = 0
            for j in range(nb_isl_arch):
                departure += islands[1 + i * nb_isl_arch + j].departure * x_vars[1 + i * nb_isl_arch + j]
                arrival += islands[1 + (i+1) * nb_isl_arch + j].arrival * x_vars[1 + (i+1) * nb_isl_arch + j]
            couple_max_duration_mid.append(arrival - departure)
        for i in range(nb_isl_arch):
            couple_max_duration_end.append((islands[nb_islands - 1].arrival - islands[1 + (nb_arch-1) * nb_isl_arch + i].departure) * x_vars[1 + (nb_arch-1) * nb_isl_arch + i])

        prob += lpSum(couple_max_duration_st) <= x_vars_min_sail[0]
        prob += lpSum(couple_max_duration_end) <= x_vars_min_sail[1]
        for i in range(nb_arch-1):
            prob += couple_max_duration_mid[i] <= x_vars_min_sail[i+2]
        
        prob += lpSum(x_vars_min_sail)
        

    if optimization_cost == OptimizationCost.min_max_sailing_time:

        x_list_min_max_sail = [i for i in range(nb_arch+1)]
        x_vars_min_max_sail = LpVariable.dicts("Min_max_sail", x_list_min_max_sail, lowBound=0)
        x_vars_max = LpVariable.dicts("Min_max_sail", [0], lowBound=0)

        couple_max_duration_st = []
        couple_max_duration_mid = []
        couple_max_duration_end = []
        
        for i in range(nb_isl_arch):
            couple_max_duration_st.append(islands[i+1].arrival * x_vars[i+1] - islands[0].departure * x_vars[i+1])
        
        for i in range(nb_arch-1):
            departure = 0
            arrival = 0
            for j in range(nb_isl_arch):
                departure += islands[1 + i * nb_isl_arch + j].departure * x_vars[1 + i * nb_isl_arch + j]
                arrival += islands[1 + (i+1) * nb_isl_arch + j].arrival * x_vars[1 + (i+1) * nb_isl_arch + j]
            couple_max_duration_mid.append(arrival - departure)
        for i in range(nb_isl_arch):
            couple_max_duration_end.append((islands[nb_islands - 1].arrival - islands[1 + (nb_arch-1) * nb_isl_arch + i].departure) * x_vars[1 + (nb_arch-1) * nb_isl_arch + i])

        prob += lpSum(couple_max_duration_st) <= x_vars_min_max_sail[0]
        prob += lpSum(couple_max_duration_end) <= x_vars_min_max_sail[0]
        for i in range(nb_arch-1):
            prob += couple_max_duration_mid[i] <= x_vars_min_max_sail[0]

        #for i in range(len(x_vars_min_max_sail)):
        #    prob += x_vars_min_max_sail[i] < x_vars_max[0]
        
        prob += x_vars_min_max_sail[0]
    
    # LISTS


    # CONSTRAINTS



    for i in range(nb_arch):
        prob += lpSum([x_vars[1 + i*nb_isl_arch + a] for a in range(nb_isl_arch)]) == 1

    if min_nights_individual_island is not None:
        for i in range(nb_arch):
            prob += lpSum([islands[1 + i*nb_isl_arch + a].nights * x_vars[1 + i*nb_isl_arch + a] for a in range(nb_isl_arch)]) >= min_nights_individual_island

    if min_total_crew is not None:
        for i in range(nb_arch):
            isl_range = (i+1)*nb_isl_arch
            prob += lpSum([islands[a+1].delta_crew * x_vars[a+1] for a in range(isl_range)]) >= (min_total_crew - start_crew)

    if max_total_crew is not None:
        for i in range(nb_arch):
            isl_range = (i+1)*nb_isl_arch
            prob += lpSum([islands[a+1].delta_crew * x_vars[a+1] for a in range(isl_range)]) <= (max_total_crew - start_crew)

    if max_duration_individual_journey is not None:

        couple_max_duration_st = []
        couple_max_duration_mid = []
        couple_max_duration_end = []
        
        for i in range(nb_isl_arch):
            couple_max_duration_st.append(islands[i+1].arrival * x_vars[i+1] - islands[0].departure * x_vars[i+1])
        for i in range(nb_arch-1):
            departure = 0
            arrival = 0
            for j in range(nb_isl_arch):
                departure += islands[1 + i * nb_isl_arch + j].departure * x_vars[1 + i * nb_isl_arch + j]
                arrival += islands[1 + (i+1) * nb_isl_arch + j].arrival * x_vars[1 + (i+1) * nb_isl_arch + j]
            couple_max_duration_mid.append(arrival - departure)
        for i in range(nb_isl_arch):
            couple_max_duration_end.append((islands[nb_islands - 1].arrival - islands[1 + (nb_arch-1) * nb_isl_arch + i].departure) * x_vars[1 + (nb_arch-1) * nb_isl_arch + i])

        prob += lpSum(couple_max_duration_st) <= max_duration_individual_journey
        prob += lpSum(couple_max_duration_end) <= max_duration_individual_journey
        for i in range(nb_arch-1):
            prob += couple_max_duration_mid[i] <= max_duration_individual_journey

    if max_L1_distance_individual_journey is not None:

        l1_dist_st = 0
        for i in range(nb_isl_arch):
            l1_dist_st += (np.abs(islands[i+1].y - islands[0].y) + np.abs(islands[i+1].x - islands[0].x)) * x_vars[i+1]
        prob += l1_dist_st <= max_L1_distance_individual_journey

        l1_dist_end = 0

        for i in range(nb_isl_arch):
            l1_dist_end += (np.abs(islands[1 + (nb_arch-1) * nb_isl_arch + i].y - islands[nb_islands - 1].y) + np.abs(islands[1 + (nb_arch-1) * nb_isl_arch + i].x - islands[nb_islands - 1].x)) * x_vars[1 + (nb_arch-1) * nb_isl_arch + i]
        prob += l1_dist_end <= max_L1_distance_individual_journey

        for i in range(nb_arch-1):

            l1_x1 = 0
            l1_y1 = 0
            l1_x2 = 0
            l1_y2 = 0
            l1_dist_1 = 0
            l1_dist_2 = 0
            l1_dist_3 = 0
            l1_dist_4 = 0

            for j in range(nb_isl_arch):

                l1_x1 += islands[1 + i * nb_isl_arch + j].x * x_vars[1 + i * nb_isl_arch + j]
                l1_y1 += islands[1 + i * nb_isl_arch + j].y * x_vars[1 + i * nb_isl_arch + j]
                l1_x2 += islands[1 + (i+1) * nb_isl_arch + j].x * x_vars[1 + (i+1) * nb_isl_arch + j]
                l1_y2 += islands[1 + (i+1) * nb_isl_arch + j].y * x_vars[1 + (i+1) * nb_isl_arch + j]


            l1_dist_1 = (l1_y2 - l1_y1) + (l1_x2 - l1_x1)
            l1_dist_2 = (l1_y2 - l1_y1) + (l1_x1 - l1_x2)
            l1_dist_3 = (l1_y1 - l1_y2) + (l1_x2 - l1_x1)
            l1_dist_4 = (l1_y1 - l1_y2) + (l1_x1 - l1_x2)

            prob += l1_dist_1 <= x_vars_L1_abs[i]
            prob += l1_dist_2 <= x_vars_L1_abs[i]
            prob += l1_dist_3 <= x_vars_L1_abs[i]
            prob += l1_dist_4 <= x_vars_L1_abs[i]
            prob += x_vars_L1_abs[i] <= max_L1_distance_individual_journey


    prob.solve()
    print("Status:", LpStatus[prob.status])

    if LpStatus[prob.status]=="Optimal":
        feasibility = Feasibility.feasible
    else:
        feasibility = Feasibility.unfeasible

    voyage_plan = [0]

    for i in range(nb_islands-2):
        if value(x_vars[i+1])==1:
            voyage_plan.append(i+1)

    voyage_plan.append(nb_islands-1)

    return SolutionVoyage(feasibility, voyage_plan)
