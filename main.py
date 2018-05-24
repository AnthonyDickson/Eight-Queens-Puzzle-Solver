from eightpuzzle import eightpuzzle
from math import sqrt
import argparse
import time


class Node:
    def __init__(self, s, parent, action, cost, h):
        self.s = s
        self.parent = parent
        self.action = action
        self.cost = cost
        self.h = h

    def total_cost(self):
        return self.cost + self.h


class EightPuzzleSolver:
    def __init__(self, puzzle, verbose=False):
        self.verbose = verbose
        self.puzzle = puzzle
        self.num_cells = len(puzzle.reset())
        self.row_size = int(sqrt(self.num_cells))

    def print(self, msg):
        if self.verbose:
            print(msg)

    def h(self, s):
        # calculate the manhattan distance
        self.print('\nCalculating h(n) for {}'.format(s))

        score = 0

        for i in range(self.num_cells):
            self.print('Puzzle Piece {}:'.format(i))

            goal_row = i // self.row_size
            goal_col = i % self.row_size
            self.print('\tgoal position is: {}, {}'.format(goal_row, goal_col))

            pos = s[i]
            curr_row = pos // self.row_size
            curr_col = pos % self.row_size
            self.print('\tcurrently at: {}, {}'.format(curr_row, curr_col))

            dy = abs(curr_row - goal_row)
            dx = abs(curr_col - goal_col)
            distance = dx + dy
            self.print('\tdistance from goal: {} ({}, {})'.format(distance, dx, dy))

            score += distance

        self.print('Heuristic for {}: {}\n'.format(s, score))

        return score

    def solve(self):
        """Solve the eight puzzle. Returns turns taken and the solution path as
        a tuple.
        """
        init_state = self.puzzle.reset()
        root = Node(s=init_state, action=None, parent=None, cost=0, h=self.h(init_state))
        open_set = list()
        open_set.append(root)
        closed_set = list()

        while len(open_set) > 0:
            current = open_set[0]

            for node in open_set:
                if node.total_cost() < current.total_cost():
                    current = node

            open_set.remove(current)
            closed_set.append(current.s)

            if self.puzzle.isgoal(current.s):
                print('Solution found!')
                return current.cost, self.action_list(current)

            for action in self.puzzle.actions(s=current.s):
                next_state = self.puzzle.step(s=current.s, a=action)
                child = Node(s=next_state, action=action, parent=current, cost=current.cost + 1, h=self.h(next_state))

                if next_state in closed_set:
                    continue

                if child not in open_set:
                    open_set.append(child)

        print('Solution not found.')
        return -1, []

    @staticmethod
    def action_list(node):
        actions = []

        while node is not None:
            actions.append(node.action)
            node = node.parent

        return list(reversed(actions))


def main():
    parser = argparse.ArgumentParser(description='Solves the eight puzzle.')
    parser.add_argument('-m', '--mode', default='easy', choices=['easy', 'medium', 'hard'])
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()

    puzzle = eightpuzzle(mode=args.mode)
    start_time = time.time()

    turns, path = EightPuzzleSolver(puzzle, verbose=args.verbose).solve()

    elapsed_time = time.time() - start_time
    print("Elapsed time: {:.2f} seconds".format(elapsed_time))
    print('Number of turns taken: {}'.format(turns))
    puzzle.show(a=path)


if __name__ == '__main__':
    main()
