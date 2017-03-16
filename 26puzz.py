#
# Author:       Tyler Crawford
# File:         26puzz.py
# Description:  Finds the path from initial state to goal state of a 26 puzzle using the A* algorithm
#

import bisect
import time

# Test configurations
goal = tuple(range(27))
equal = tuple(range(27))
test_4 = (3, 1, 2, 4, 13, 5, 6, 7, 8, 9, 10, 11, 12, 14, 0, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26)
test_10 = (3, 1, 2, 4, 13, 8, 0, 16, 7, 9, 10, 11, 12, 14, 5, 6, 15, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26)
test_14 = (3, 1, 2, 4, 13, 8, 16, 15, 0, 9, 10, 11, 12, 14, 5, 6, 17, 7, 18, 19, 20, 21, 22, 23, 24, 25, 26)
test_50 = (1, 14, 2, 22, 10, 5, 15, 7, 8, 6, 0, 13, 3, 4, 11, 24, 16, 17, 19, 21, 20, 18, 9, 23, 12, 25, 26)
test_100 = (3, 1, 2, 9, 4, 7, 6, 16, 8, 10, 18, 11, 5, 17, 0, 15, 22, 25, 19, 20, 14, 12, 21, 13, 24, 26, 23)

moves = ('east', 'west', 'north', 'south', 'up', 'down')

# Dictionary containing triples of "add amounts" for row, col, lev
to_add = {
    'up': [1, 0, 0],
    'down': [-1, 0, 0],
    'north': [0, 1, 0],
    'south': [0, -1, 0],
    'east': [0, 0, -1],
    'west': [0, 0, 1]
}


def to_list(state):
    """
    Converts a tuple state into a list of lists of lists
    :param state: the state to convert
    :return: a state as a list of lists of lists
    """
    return [
        [
            [state[i] for i in range(0, 3)],
            [state[i] for i in range(3, 6)],
            [state[i] for i in range(6, 9)]
        ],
        [
            [state[i] for i in range(9, 12)],
            [state[i] for i in range(12, 15)],
            [state[i] for i in range(15, 18)]
        ],
        [
            [state[i] for i in range(18, 21)],
            [state[i] for i in range(21, 24)],
            [state[i] for i in range(24, 27)]
        ]
    ]


def show_state(list_state):
    """
    Displays a list form of a state to the console formatted nicely
    :param list_state: a list form of a state
    :return: None
    """
    for i in range(3):
        print(list_state[0][i], list_state[1][i], list_state[2][i])


def level_row_col_of(item, state):
    """
    Obtains the level row and column of an item in a state
    :param item: the value to find the level row and column for
    :param state: a tuple configuration
    :return: the level row and column of an item in a state
    """
    place = state.index(item)
    level = int(place / 9)
    place -= 9 * level
    row = int(place / 3)
    col = place - (3 * row)
    return level, row, col


def make_move(move, state):
    """
    Makes a directional move based on the state passed to the function
    :param move: a directional move
    :param state: a configuration stored as a tuple of 27 values
    :return: the state that is being moved to based on the move passed in
    """
    lrc_of_zero = level_row_col_of(0, state)
    lrc_to_add = to_add[move]
    triple_tile = []

    for i in range(3):
        triple_tile.append(lrc_of_zero[i] + lrc_to_add[i])

    tile_location = 9 * triple_tile[0] + 3 * triple_tile[1] + triple_tile[2]

    state_list = list(state)
    state_list[state.index(0)] = state_list[tile_location]
    state_list[tile_location] = 0

    return tuple(state_list)


def make_node(state, g, h, path):
    """
    Creates a new node comprised of a state as a tuple, the g, h, and f values
    derived via the A* algorithm, and the current path traveled to get to the node
    :param state: A configuration stored as a tuple of 27 values
    :param g: the total distance traveled so far
    :param h: the heuristic of how far is left to travel
    :param path: the path traveled in terms of directions so far
    :return: a new node
    """
    return {
        'state': state,
        'g': g,
        'h': h,
        'f': g + h,
        'path': path
    }


def manhattan(state1, state2):
    """
    Gets the manhattan distance between two states
    :param state1: any state as a tuple to start with
    :param state2: any state as a tuple to go to
    :return: the manhattan distance from one state to another state
    """
    distance = 0
    for i in range(1, 27):
        level1, row1, col1 = level_row_col_of(i, state1)
        level2, row2, col2 = level_row_col_of(i, state2)
        distance += abs(level1 - level2) + abs(row1 - row2) + abs(col1 - col2)
    return distance


def make_move_node(move, node, goal_state):
    """
    Makes a new node based on the move passed to the function
    :param move: a directional move
    :param node: the current node being viewed
    :param goal_state: the goal state to go to
    :return: a new node with a new state, g, h, f, and path values
    """
    new_state = make_move(move, node['state'])
    g = node['g']
    h = manhattan(new_state, goal_state)
    return {
        'state': new_state,
        'g': g + 1,
        'h': h,
        'f': g + 1 + h,
        'path': node['path'] + [move]
    }


def generate_moves(state):
    """
    Get the possible moves that a node can go to next based on a its state
    :param state: the state being viewed
    :return: a list of moves that can be made from the current state
    """
    row_moves = [['north'], ['north', 'south'], ['south']]
    col_moves = [['west'], ['east', 'west'], ['east']]
    lev_moves = [['up'], ['up', 'down'], ['down']]

    lev, row, col = level_row_col_of(0, state)
    return lev_moves[lev] + row_moves[row] + col_moves[col]


def expand_node(node, goal_state, visited_states):
    """
    Expands the current node to see where it can go next and checks to see if it would go to
    a node that is already visited and skip that if so
    :param node: the current node being looked at
    :param goal_state: the goal state
    :param visited_states: a set of states that have been visited by the algorithm
    :return: a list of possible nodes to move to next
    """
    l = []
    for move in generate_moves(node['state']):
        amove = make_move_node(move, node, goal_state)
        if amove['state'] not in visited_states:
            l.append(amove)
    return l


def add_good_nodes(new_nodes, wl, wl_fvals):
    """
    Add the good nodes to the waiting list
    :param new_nodes: the new nodes to check
    :param wl: the waiting list
    :param wl_fvals: the waiting list of f values
    :return: an updated waiting list and waiting list of f values
    """
    for node in new_nodes:
        wls = [n['state'] for n in wl]
        if node['state'] in wls:
            node = keep_if_better(node, wl, wls)
        if node:
            wl, wl_fvals = insert_keyed(node, node['f'], wl, wl_fvals)

    return wl, wl_fvals


def insert_keyed(node, f, alist, keys_list):
    """
    Inserts a node into the waiting list ordered by the f value
    :param node: a node to insert into the waiting list
    :param f: the f value of the node
    :param alist: the waiting list
    :param keys_list: a waiting list of f values
    :return: the newly sorted waiting list and waiting list of f values
    """
    place = bisect.bisect_left(keys_list, f)
    return alist[:place] + [node] + alist[place:], keys_list[:place] + [f] + keys_list[place:]


def keep_if_better(node, wl, wls):
    """
    Keeps the node with the better g value if there is more than one
    :param node: a current configuration stored as a dictionary
    :param wl: the waiting list of nodes
    :param wls: the waiting list of states store as tuples
    :return: the node passed if its g value is less than the old node or an empty list if not
    """
    index = wls.index(node['state'])
    old_node = wl[index]
    return node if node['g'] < old_node['g'] else []


def find_path(init_state, goal_state):
    """
    Runs the A* algorithm to go from initial state to goal state
    :param init_state: the initial starting state
    :param goal_state: the goal state to finish at
    :return: the path from initial to goal state in terms of directions
    """
    print('Initial state:', init_state, "\n")

    wl = []
    wl_fvals = []
    visited_states = set()

    distance = manhattan(init_state, goal_state)
    init_node = make_node(init_state, 0, distance, [])
    current_node = init_node

    while current_node['state'] != goal_state:
        visited_states.add(current_node['state'])
        new_nodes = expand_node(current_node, goal_state, visited_states)
        wl, wl_fvals = add_good_nodes(new_nodes, wl, wl_fvals)
        current_node = wl[0]
        wl.remove(current_node)
        wl_fvals.remove(wl_fvals[0])

    return current_node['path']


def show_path(init_state, path):
    """
    Shows the path from initial state to goal state with the moves made
    :param init_state: the initial state
    :param path: the path with directions from initial state to goal state
    :return: 'Return done traveling message'
    """
    show_state(to_list(init_state))
    old_state = init_state
    for move in path:
        print()
        print(move)
        new_state = make_move(move, old_state)
        show_state(to_list(new_state))
        old_state = new_state
    return 'Done traveling'


def main():
    """
    Main testing function
    :return: None
    """

    initstate = test_50

    print("Manhattan:", manhattan(initstate, goal))

    # Get the path from initial state to goal state recording the time it took to complete
    time1 = time.time()
    path = find_path(initstate, goal)
    time2 = time.time()

    print("Path to goal state:", path)
    print("Number of moves to goal state:", len(path))

    totaltime = time2 - time1
    print("Time to complete:", "%.4f" % totaltime, "seconds")

    print("\n" + show_path(initstate, path))


if __name__ == '__main__':
    main()
