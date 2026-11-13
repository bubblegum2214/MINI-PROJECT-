"""
CS3810 Mini-Project 1 - Part 2: The Search Algorithms (100 points)
==================================================================

Implement dfs_search, astar_search, and idastar_search below. Do not change
the signatures or the return shapes: run_tests.py and the grading harness
unpack them exactly as documented.

You may NOT use a library implementation of DFS, A*, or IDA* (networkx,
simpleai, aima-python, ...). Using heapq, collections.deque, and the
provided PriorityQueue is expected and fine.

Metric definitions - use these, and say which you used in your report:

    nodes_expanded      A node is EXPANDED when it is removed from the
                        frontier and its successors are generated. Do not
                        count nodes that were merely generated.

    max_frontier_size   The largest number of live entries the frontier
                        ever held. For the PriorityQueue helper this is
                        len(queue), not len(queue.heap).

    iterations          (IDA* only) The number of depth-limited passes,
                        i.e. how many times the f-cost threshold was set.
                        A search that succeeds on the first threshold has
                        iterations == 1.

Suggested order of work: DFS first, then A*, then IDA*.
"""

import math

from priority_queue import PriorityQueue

# Sentinel used by the IDA* recursion to report success. Returning a plain
# number means "the smallest f-value I saw above the threshold".
FOUND = 'FOUND'


def dfs_search(problem):
    """
    Perform Depth-First Search.

    Args:
        problem: VacuumWorld instance

    Returns:
        Tuple (solution_path, nodes_expanded, max_frontier_size)
        solution_path: List of actions, or None if no solution
        nodes_expanded: Number of nodes expanded during search
        max_frontier_size: Maximum size of frontier during search

    Requirements:
        * Iterative, with an explicit stack. Do NOT recurse - you will hit
          Python's recursion limit on the larger grids.
        * Cycle detection with an explored set, or DFS will not terminate.
        * Returns the FIRST solution found. It will not be optimal, and it
          is not supposed to be.

    Hint: push (state, path_so_far) pairs. Push successors in reversed()
    order if you want the stack to explore them in ACTION_ORDER order.
    """
    start = problem.initial_state()
    stack = [(start, [])]
    explored = set()
    nodes_expanded = 0
    max_frontier_size = 1

    while stack:
        state, path = stack.pop()

        if state in explored:
            continue

        if problem.is_goal(state):
            return path, nodes_expanded, max_frontier_size

        explored.add(state)
        nodes_expanded += 1

        for action in reversed(problem.get_actions(state)):
            next_state = problem.result(state, action)

            if next_state not in explored:
                stack.append((next_state, path + [action]))

        max_frontier_size = max(max_frontier_size, len(stack))

    return None, nodes_expanded, max_frontier_size



def astar_search(problem, heuristic):
    """
    Perform A* Search.

    Args:
        problem: VacuumWorld instance
        heuristic: Function h(state, problem) -> estimated cost to goal

    Returns:
        Tuple (solution_path, nodes_expanded, max_frontier_size)

    Requirements:
        * Priority queue ordered by f(n) = g(n) + h(n).
        * Handle REOPENING: if you find a cheaper path to a state you have
          already expanded, you must be able to improve it. The provided
          PriorityQueue supports this - pushing an item already in the
          queue replaces its priority instead of duplicating it.
        * With an admissible heuristic this MUST return an optimal
          solution. run_tests.py checks that against known optimal costs.

    Hint: keep a dict g[state] of best-known cost-so-far and a dict
    came_from[state] = (parent_state, action) to rebuild the path at the
    end. A helper like _reconstruct() below keeps the main loop readable.
    """
    start = problem.initial_state()
    frontier = PriorityQueue()
    frontier.push(start, heuristic(start, problem))

    g = {start: 0}
    came_from = {}
    nodes_expanded = 0
    max_frontier_size = 1

    while frontier:
        state = frontier.pop()

        if problem.is_goal(state):
            path = _reconstruct(came_from, state)
            return path, nodes_expanded, max_frontier_size

        nodes_expanded += 1

        for action in problem.get_actions(state):
            next_state = problem.result(state, action)
            new_cost = g[state] + problem.action_cost(state, action)

            if next_state not in g or new_cost < g[next_state]:
                g[next_state] = new_cost
                came_from[next_state] = (state, action)
                priority = new_cost + heuristic(next_state, problem)
                frontier.push(next_state, priority)

        max_frontier_size = max(max_frontier_size, len(frontier))

    return None, nodes_expanded, max_frontier_size


def _reconstruct(came_from, state):
    """Walk came_from backwards from `state` and return the action list.

    Args:
        came_from: Dict mapping state -> (parent_state, action)
        state: The goal state reached by the search

    Returns:
        List of actions from the initial state to `state`.
    """
    path = []

    while state in came_from:
        parent, action = came_from[state]
        path.append(action)
        state = parent

    path.reverse()
    return path


def idastar_search(problem, heuristic):
    """
    Perform Iterative Deepening A* Search.

    Args:
        problem: VacuumWorld instance
        heuristic: Function h(state, problem) -> estimated cost to goal

    Returns:
        Tuple (solution_path, nodes_expanded, iterations)
        iterations: Number of depth-limited iterations performed

    Requirements:
        * Iterative deepening on an f-cost THRESHOLD, not on depth. The
          next threshold is the smallest f-value that exceeded the current
          one.
        * Linear space: no explored set carried across iterations. You may
          track the states on the current path to avoid immediate cycles.
        * Returns an optimal solution.

    Structure that works (write DFS and A* first - this will make far more
    sense once you have both):

        threshold = h(start)
        loop:
            result = search(start, g=0, threshold)
            if result is FOUND:    return the path
            if result is infinite: return None (no solution)
            threshold = result

    where search(state, g, threshold) returns FOUND, or the smallest
    f-value it saw that exceeded the threshold, or math.inf.
    """
    
    start = problem.initial_state()
    threshold = heuristic(start, problem)
    iterations = 0
    nodes_expanded = 0

    while True:
        iterations += 1
        path = []
        path_states = {start}

        def search(state, g):
            nonlocal nodes_expanded

            f = g + heuristic(state, problem)

            if f > threshold:
                return f

            if problem.is_goal(state):
                return FOUND

            nodes_expanded += 1
            minimum = math.inf

            for action in problem.get_actions(state):
                next_state = problem.result(state, action)

                if next_state in path_states:
                    continue

                path_states.add(next_state)
                path.append(action)

                result = search(
                    next_state,
                    g + problem.action_cost(state, action)
                )

                if result == FOUND:
                    return FOUND

                minimum = min(minimum, result)

                path.pop()
                path_states.remove(next_state)

            return minimum

        result = search(start, 0)

        if result == FOUND:
            return path, nodes_expanded, iterations

        if result == math.inf:
            return None, nodes_expanded, iterations

        threshold = result



if __name__ == "__main__":
    # Quick manual check once you have implemented an algorithm:
    from test_grids import EXAMPLE, parse_grid
    from vacuum_world import VacuumWorld
    from heuristics import h2

    grid, start, dirty = parse_grid(EXAMPLE)
    problem = VacuumWorld(grid, start, dirty)

    path, expanded, frontier = astar_search(problem, h2)
    print("A* on the example grid (optimal cost is 14)")
    print("  cost     :", len(path) if path else None)
    print("  expanded :", expanded)
    print("  frontier :", frontier)
    print("  plan     :", path)
