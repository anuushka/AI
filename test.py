import numpy as np
from collections import deque
from copy import deepcopy

BLUE = 1
RED = 2


class Problem:
    def __init__(self, initial):
        self.initial = initial

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def goal_test(self, state):
        raise NotImplementedError

    def path_cost(self, c):
        return c + 1


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<{}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        next_state = problem.result(deepcopy(self.state), action)
        next_node = Node(next_state, self, action,
                         problem.path_cost(self.path_cost))
        return next_node

    def solution(self):
        return [node.action for node in self.path()[1:]]

    def path(self):
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)


class ColorGame(Problem):

    def __init__(self, initial):
        super().__init__(initial)

    def result(self, state, action):
        state[0][state[1]][state[2]] = action
        if state[1] < 2:
            state[1] += 1
        else:
            if state[2] < 2:
                state[2] += 1
                state[1] = 0
        return deepcopy(state)

    def goal_test(self, state):
        # First pass
        # for i in range(3):
        #     for j in range(3):
        #         box_color = state[i*3][j*3]
        #         for m in range(3):
        #             for n in range(3):
        #                 if state[i+m][j+n] != box_color:
        #                     return False
        # Second pass
        if (state[0][1] == state[0][0]) or (state[0][1] == state[1][1]) or \
                (state[0][1] == state[0][2]):
            return False
        if (state[1][0] == state[0][0]) or (state[1][0] == state[1][1]) or \
                (state[1][0] == state[2][0]):
            return False
        if (state[1][2] == state[2][2]) or (state[1][2] == state[1][1]) or \
                (state[1][2] == state[0][2]):
            return False
        if (state[2][1] == state[2][0]) or (state[2][1] == state[1][1]) or \
                (state[2][1] == state[2][2]):
            return False
        return True

    def actions(self, state):
        # print(state[1], state[2])
        if (state[1] <= 2) or (state[2] <= 2):
            return [BLUE, RED]
        return []


def breadth_first_tree_search(problem):
    frontier = deque([Node(problem.initial)])  # FIFO queue
    while frontier:
        node = frontier.popleft()  # First element
        # print(node.action)
        if problem.goal_test(node.state[0]):
            return node
        print(node.depth)
        frontier.extend(node.expand(problem))
    return None


def depth_first_tree_search(problem):
    frontier = [Node(problem.initial)]  # Stack
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state[0]):
            return node
        frontier.extend(node.expand(problem))
    return None


init = [np.zeros((3, 3)), 0, 0]

gnode = breadth_first_tree_search(ColorGame(deepcopy(init)))
print(gnode)
print(gnode.solution())
print(Node.path(gnode))

