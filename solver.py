import sys
import puzz
import pdqpq


GOAL_STATE = puzz.EightPuzzleBoard("012345678")
GOAL = '012345678'
# state = puzz.EightPuzzleBoard('123457608')
# print(state)
# print(state.pretty())
# succs = state.successors()
# print(succs)

def solve_puzzle(start_state, flavor):
    """Perform a search to find a solution to a puzzle.
    
    Args:
        start_state (EightPuzzleBoard): the start state for the search
        flavor (str): tag that indicate which type of search to run.  Can be one of the following:
            'bfs' - breadth-first search
            'ucost' - uniform-cost search
            'greedy-h1' - Greedy best-first search using a misplaced tile count heuristic
            'greedy-h2' - Greedy best-first search using a Manhattan distance heuristic
            'greedy-h3' - Greedy best-first search using a weighted Manhattan distance heuristic
            'astar-h1' - A* search using a misplaced tile count heuristic
            'astar-h2' - A* search using a Manhattan distance heuristic
            'astar-h3' - A* search using a weighted Manhattan distance heuristic
    
    Returns: 
        A dictionary containing describing the search performed, containing the following entries:
        'path' - list of 2-tuples representing the path from the start to the goal state (both 
            included).  Each entry is a (str, EightPuzzleBoard) pair indicating the move and 
            resulting successor state for each action.  Omitted if the search fails.
        'path_cost' - the total cost of the path, taking into account the costs associated with 
            each state transition.  Omitted if the search fails.
        'frontier_count' - the number of unique states added to the search frontier at any point 
            during the search.
        'expanded_count' - the number of unique states removed from the frontier and expanded 
            (successors generated)

    """
    print('type of start state', type(start_state))
    if flavor.find('-') > -1:
        strat, heur = flavor.split('-')
    else:
        strat, heur = flavor, None

    if strat == 'bfs':
        return BreadthFirstSolver().solve(start_state)
    elif strat == 'ucost':
        return UniformCostSearch().solve(start_state)
    elif strat == 'greedy':
        return GreedyBestFirstSearch().solve(start_state, heur)
    elif strat == 'astar':
        return AStarSearch().solve(start_state, heur)
    else:
        raise ValueError("Unknown search flavor '{}'".format(flavor))


class BreadthFirstSolver:
    """Implementation of Breadth-First Search based puzzle solver"""

    def __init__(self):
        self.goal = GOAL_STATE
        self.parents = {}  # state -> parent_state
        self.frontier = pdqpq.FifoQueue()
        self.explored = set()
        self.frontier_count = 0  # increment when we add something to frontier
        self.expanded_count = 0  # increment when we pull something off frontier and expand
    
    def solve(self, start_state):
        """Carry out the search for a solution path to the goal state.
        
        Args:
            start_state (EightPuzzleBoard): start state for the search 
        
        Returns:
            A dictionary describing the search from the start state to the goal state.

        """
        self.parents[start_state] = None
        self.add_to_frontier(start_state)

        if start_state == self.goal:  # edge case        
            return self.get_results_dict(start_state)

        while not self.frontier.is_empty():
            node = self.frontier.pop()  # get the next node in the frontier queue
            succs = self.expand_node(node)

            for move, succ in succs.items():
                if (succ not in self.frontier) and (succ not in self.explored):
                    self.parents[succ] = node

                    # BFS checks for goal state _before_ adding to frontier
                    if succ == self.goal:
                        return self.get_results_dict(succ)
                    else:
                        self.add_to_frontier(succ)

        # if we get here, the search failed
        return self.get_results_dict(None) 

    def add_to_frontier(self, node):
        """Add state to frontier and increase the frontier count."""
        self.frontier.add(node)
        self.frontier_count += 1

    def expand_node(self, node):
        """Get the next state from the frontier and increase the expanded count."""
        self.explored.add(node)
        self.expanded_count += 1
        return node.successors()

    def get_results_dict(self, state):
        """Construct the output dictionary for solve_puzzle()
        
        Args:
            state (EightPuzzleBoard): final state in the search tree
        
        Returns:
            A dictionary describing the search performed (see solve_puzzle())

        """
        results = {}
        results['frontier_count'] = self.frontier_count
        results['expanded_count'] = self.expanded_count
        if state:
            results['path_cost'] = self.get_cost(state)
            path = self.get_path(state)
            moves = ['start'] + [ path[i-1].get_move(path[i]) for i in range(1, len(path)) ]
            results['path'] = list(zip(moves, path))
        print('number of moves: ', len(results['path']) - 1)
        return results

    def get_path(self, state):
        """Return the solution path from the start state of the search to a target.
        
        Results are obtained by retracing the path backwards through the parent tree to the start
        state for the serach at the root.
        
        Args:
            state (EightPuzzleBoard): target state in the search tree
        
        Returns:
            A list of EightPuzzleBoard objects representing the path from the start state to the
            target state

        """
        path = []
        while state is not None:
            path.append(state)
            state = self.parents[state]
        path.reverse()
        return path

    def get_cost(self, state): 
        """Calculate the path cost from start state to a target state.
        
        Transition costs between states are equal to the square of the number on the tile that 
        was moved. 

        Args:
            state (EightPuzzleBoard): target state in the search tree
        
        Returns:
            Integer indicating the cost of the solution path

        """
        cost = 0
        path = self.get_path(state)
        for i in range(1, len(path)):
            x, y = path[i-1].find(None)  # the most recently moved tile leaves the blank behind
            tile = path[i].get_tile(x, y)        
            cost += int(tile)**2
        return cost

class UniformCostSearch(BreadthFirstSolver):

    def __init__(self):
        self.goal = GOAL_STATE
        self.parents = {}  # state -> parent_state
        self.frontier = pdqpq.PriorityQueue()
        self.explored = set()
        self.frontier_count = 0  # increment when we add something to frontier
        self.expanded_count = 0  # increment when we pull something off frontier and expand

    def add_to_frontier(self, node, cost):
        """Add state to frontier and increase the frontier count."""
        #print(node)
        self.frontier.add(node, cost)
        self.frontier_count += 1

    # We need to redefine the solve method to use UCS instead of BFS
    def solve(self, start_state):
        self.parents[start_state] = None
        self.add_to_frontier(start_state, 0)

        if start_state == self.goal:  # edge case
            return self.get_results_dict(start_state)

        while not self.frontier.is_empty():
            node = self.frontier.pop()  # pop off the front node in the priority queue
            succs = self.expand_node(node)  # get the successors of the node

            if node == self.goal:  # check if popped node is our goal state
                return self.get_results_dict(node)  # return solution

            for move, succ in succs.items():  # looping through all the successors to our current state
                if (succ not in self.frontier) and (succ not in self.explored):  # if node is not in frontier or explored
                    self.parents[succ] = node  # define parent nodes
                    self.add_to_frontier(succ, self.get_cost(succ))  # add it to the frontier
                elif succ in self.frontier:  # if the node is already in the frontier
                    temp_parent = self.parents[succ]  # temporary storage of parents
                    self.parents[succ] = node  # set new parents of succ
                    if self.get_cost(succ) < self.frontier.get(succ):  # if the cost to the same node is now less
                        self.frontier.add(succ, self.get_cost(succ))  # update the priority queue
                    else:  # if the cost of the node on the queue is less than this new path
                        self.parents[succ] = temp_parent  # We do not change the queue so we readjust succ's parents
        return self.get_results_dict(None)# if we get here, the search failed

class GreedyBestFirstSearch(BreadthFirstSolver):

    def __init__(self):
        self.goal = GOAL_STATE
        self.parents = {}  # state -> parent_state
        self.frontier = pdqpq.PriorityQueue()
        self.explored = set()
        self.frontier_count = 0  # increment when we add something to frontier
        self.expanded_count = 0  # increment when we pull something off frontier and expand

    def add_to_frontier(self, node, cost):
        """Add state to frontier and increase the frontier count."""
        #print(node)
        self.frontier.add(node, cost)
        self.frontier_count += 1

    def h1(self, state):
        count = 0
        for x in range (0, 3):  # loop through x coordinates
            for y in range(0, 3):  # loop through y coordinates
                if state.get_tile(x, y) == 0:  # do not account for blank tile
                    continue
                else:
                    if state.get_tile(x, y) != self.goal.get_tile(x, y):  # if the tiles at positions x, y are not the same
                        count = count + 1
        return count

    def h2(self, state):
        # state is of the type 8 puzzle board
        # dictionary containing the piece and ideal position in the goal state
        ideal_board = {0: (0, 2), 1: (1, 2), 2: (2, 2), 3: (0, 1), 4: (1, 1), 5: (2, 1), 6: (0, 0), 7: (1, 0), 8: (2, 0)}
        count = 0

        for x in range (0, 3):  # loop through x coordinates
            for y in range(0, 3):  # loop through y coordinates
                if state.get_tile(x, y) == 0:  # do not account for blank tile
                    continue
                else:
                    if state.get_tile(x, y) != self.goal.get_tile(x, y):  # if the tiles at positions x, y are not the same
                        ideal_x = ideal_board[int(state.get_tile(x, y))][0]
                        ideal_y = ideal_board[int(state.get_tile(x, y))][1]
                        count = count + (abs(x - ideal_x) + abs(y - ideal_y))
        return count

    def h3(self, state):  # we just need to multiply the value of the tile square to our current value

        ideal_board = {0: (0, 2), 1: (1, 2), 2: (2, 2), 3: (0, 1), 4: (1, 1), 5: (2, 1), 6: (0, 0), 7: (1, 0), 8: (2, 0)}
        count = 0

        for x in range (0, 3):  # loop through x coordinates
            for y in range(0, 3):  # loop through y coordinates
                if state.get_tile(x, y) == 0:  # do not account for blank tile
                    continue
                else:
                    if state.get_tile(x, y) != self.goal.get_tile(x, y):  # if the tiles at positions x, y are not the same
                        ideal_x = ideal_board[int(state.get_tile(x, y))][0]
                        ideal_y = ideal_board[int(state.get_tile(x, y))][1]
                        count = count + (abs(x - ideal_x) + abs(y - ideal_y)) * int(state.get_tile(x, y))**2
        return count

    def solve(self, start_state, h):
        self.parents[start_state] = None
        self.add_to_frontier(start_state, 0)

        if start_state == self.goal:  # edge case
            return self.get_results_dict(start_state)

        def heuristic(h, succ):
            cost = 0
            if h == 'h1':  # determine cost through the desired heuristic
                cost = self.h1(succ)
            if h == 'h2':
                cost = self.h2(succ)
            if h == 'h3':
                cost = self.h3(succ)
            return cost

        while not self.frontier.is_empty():
            node = self.frontier.pop()  # pop off the front node in the priority queue

            succs = self.expand_node(node)  # get the successors of the node

            if node == self.goal:  # check if popped node is our goal state
                return self.get_results_dict(node)

            for move, succ in succs.items():  # looping through all the successors to our current state
                cost = heuristic(h, succ)

                if (succ not in self.frontier) and (succ not in self.explored):  # if successor is not in the frontier
                    self.parents[succ] = node  # define parent nodes
                    self.add_to_frontier(succ, cost)  # add it to the frontier

                elif succ in self.frontier:  # if successor is in the frontier
                    temp_parent = self.parents[succ]  # temporary storage of parents
                    self.parents[succ] = node  # set new parents of succ
                    if cost < self.frontier.get(succ):  # if the cost to the same node is less
                        self.frontier.add(succ, cost)  # update the priority queue

                    else:  # if the cost of the node on the queue is less than this new path
                        self.parents[succ] = temp_parent  # We do not change the queue so we readjust succ's parents


        return self.get_results_dict(None)  # if we get here, the search failed

class AStarSearch(BreadthFirstSolver):

    def __init__(self):
        self.goal = GOAL_STATE
        self.parents = {}  # state -> parent_state
        self.frontier = pdqpq.PriorityQueue()
        self.explored = set()
        self.frontier_count = 0  # increment when we add something to frontier
        self.expanded_count = 0  # increment when we pull something off frontier and expand

    def add_to_frontier(self, node, cost):
        """Add state to frontier and increase the frontier count."""
        #print(node)
        self.frontier.add(node, cost)
        self.frontier_count += 1

    def h1(self, state):
        count = 0
        for x in range(0, 3):  # loop through x coordinates
            for y in range(0, 3):  # loop through y coordinates
                if state.get_tile(x, y) != 0:  # do not account for blank tile
                    continue
                else:
                    if state.get_tile(x, y) != self.goal.get_tile(x, y):  # if the tiles at positions x, y are not the same
                        count = count + 1
        return count

    def h2(self, state):
        # state is of the type 8 puzzle board
        # dictionary containing the piece and ideal position in the goal state
        count = 0
        ideal_board = {0: (0, 2), 1: (1, 2), 2: (2, 2), 3: (0, 1), 4: (1, 1), 5: (2, 1), 6: (0, 0), 7: (1, 0), 8: (2, 0)}
        for x in range(0, 3):  # loop through x coordinates
            for y in range(0, 3):  # loop through y coordinates
                if state.get_tile(x, y) == 0:  # do not account for blank tile
                    continue
                else:
                    if state.get_tile(x, y) != self.goal.get_tile(x, y):  # if the tiles at positions x, y are not the same
                        ideal_x = ideal_board[int(state.get_tile(x, y))][0]
                        ideal_y = ideal_board[int(state.get_tile(x, y))][1]
                        count = count + (abs(ideal_x - x) + abs(ideal_y - y))
        return count

    def h3(self, state):  # we just need to multiply the value of the tile square to our current value
        count = 0
        ideal_board = {0: (0, 2), 1: (1, 2), 2: (2, 2), 3: (0, 1), 4: (1, 1), 5: (2, 1), 6: (0, 0), 7: (1, 0), 8: (2, 0)}

        for x in range (0, 3):  # loop through x coordinates
            for y in range(0, 3):  # loop through y coordinates
                if state.get_tile(x, y) == 0:  # do not account for blank tile
                    continue
                else:
                    if state.get_tile(x, y) != self.goal.get_tile(x, y):  # if the tiles at positions x, y are not the same
                        ideal_x = ideal_board[int(state.get_tile(x, y))][0]
                        ideal_y = ideal_board[int(state.get_tile(x, y))][1]
                        count = count + (abs(x - ideal_x) + abs(y - ideal_y)) * int(state.get_tile(x, y))**2
        return count

    def solve(self, start_state, h):  # Adjust this solve function to be AStar search - currently greedy
        self.parents[start_state] = None
        self.add_to_frontier(start_state, 0)

        if start_state == self.goal:  # edge case
            return self.get_results_dict(start_state)

        def heuristic(h, succ):
            cost = 0
            if h == 'h1':  # determine cost through the desired heuristic
                cost = self.h1(succ)
            if h == 'h2':
                cost = self.h2(succ)
            if h == 'h3':
                cost = self.h3(succ)
            return cost

        while not self.frontier.is_empty():
            node = self.frontier.pop()  # pop off the front node in the priority queue

            succs = self.expand_node(node)  # get the successors of the node

            if node == self.goal:  # check if popped node is our goal state
                return self.get_results_dict(node)  # return solution

            for move, succ in succs.items():  # looping through all the successors to our current state

                cost = heuristic(h, succ)

                if (succ not in self.frontier) and (succ not in self.explored):  # if successor is not in the frontier
                    self.parents[succ] = node  # define parent nodes
                    self.add_to_frontier(succ, self.get_cost(succ) + cost)  # add it to the frontier with cost + h value

                elif succ in self.frontier:  # if successor is in the frontier
                    temp_parent = self.parents[succ]  # temporary storage of parents
                    self.parents[succ] = node  # set new parents of succ s
                    if (self.get_cost(succ) + cost) < self.frontier.get(succ):  # if the cost to the same node is now less
                        self.frontier.add(succ, self.get_cost(succ) + cost)  # update the priority queue
                    else:  # if the cost of the node on the queue is less than this new path
                        self.parents[succ] = temp_parent  # We do not change the queue so we readjust succ's parents


def print_table(flav__results, include_path=False):
    """Print out a comparison of search strategy results.

    Args:
        flav__results (dictionary): a dictionary mapping search flavor tags result statistics. See
            solve_puzzle() for detail.
        include_path (bool): indicates whether to include the actual solution paths in the table

    """
    result_tups = sorted(flav__results.items())
    c = len(result_tups)
    na = "{:>12}".format("n/a")
    rows = [  # abandon all hope ye who try to modify the table formatting code...
        "flavor  " + "".join([ "{:>12}".format(tag) for tag, _ in result_tups]),
        "--------" + ("  " + "-"*10)*c,
        "length  " + "".join([ "{:>12}".format(len(res['path'])) if 'path' in res else na 
                                for _, res in result_tups ]),
        "cost    " + "".join([ "{:>12,}".format(res['path_cost']) if 'path_cost' in res else na 
                                for _, res in result_tups ]),
        "frontier" + ("{:>12,}" * c).format(*[res['frontier_count'] for _, res in result_tups]),
        "expanded" + ("{:>12,}" * c).format(*[res['expanded_count'] for _, res in result_tups])
    ]
    if include_path:
        rows.append("path")
        longest_path = max([ len(res['path']) for _, res in result_tups if 'path' in res ] + [0])
        print("longest", longest_path)
        for i in range(longest_path):
            row = "        "
            for _, res in result_tups:
                if len(res.get('path', [])) > i:
                    move, state = res['path'][i]
                    row += " " + move[0] + " " + str(state)
                else:
                    row += " "*12
            rows.append(row)
    print("\n" + "\n".join(rows), "\n")


def get_test_puzzles():
    """Return sample start states for testing the search strategies.
    
    Returns:
        A tuple containing three EightPuzzleBoard objects representing start states that have an
        optimal solution path length of 3-5, 10-15, and >=25 respectively.
    
    """
    five_move = puzz.EightPuzzleBoard('125348670')
    twelve_move = puzz.EightPuzzleBoard('125068437')
    twenty_seven_move = puzz.EightPuzzleBoard('342176580')

    return (five_move, twelve_move, twenty_seven_move)  # fix this line!


############################################

# if __name__ == '__main__':
#
#     # parse the command line args
#     start = puzz.EightPuzzleBoard(sys.argv[1])
#     if sys.argv[2] == 'all':
#         flavors = ['bfs', 'ucost', 'greedy-h1', 'greedy-h2',
#                    'greedy-h3', 'astar-h1', 'astar-h2', 'astar-h3']
#     else:
#         flavors = sys.argv[2:]
#
#     # run the search(es)
#     results = {}
#     for flav in flavors:
#         print("solving puzzle {} with {}".format(start, flav))
#         results[flav] = solve_puzzle(start, flav)
#
#     print_table(results, include_path=False)  # change to True to see the paths!

print(solve_puzzle(puzz.EightPuzzleBoard('123405678'), 'bfs'))
# print(solve_puzzle(puzz.EightPuzzleBoard('123405678'), 'ucost'))
# print(solve_puzzle(puzz.EightPuzzleBoard('123405678'), 'greedy-h1'))
# print(solve_puzzle(puzz.EightPuzzleBoard('123405678'), 'greedy-h2'))
# print(solve_puzzle(puzz.EightPuzzleBoard('123405678'), 'greedy-h3'))
# print(solve_puzzle(puzz.EightPuzzleBoard('123405678'), 'astar-h1'))
# print(solve_puzzle(puzz.EightPuzzleBoard('123405678'), 'astar-h2'))
# print(solve_puzzle(puzz.EightPuzzleBoard('123405678'), 'astar-h3'))

#%%

#%%
