from eightpuzzle import eightpuzzle
from math import sqrt
import argparse
import time

class Node:
	def __init__(self, s, parent, action, cost):
		self.s = s
		self.parent = parent
		self.action = action
		self.cost = cost

	def __hash__(self):
		hashcode = s.__hash__()
		hashcode += 7 * action

	def __eq__(self, other):
		if other is None:
			return False

		return self.s == other.s and self.action == other.action

class EightPuzzleSolver:
	def __init__(self, puzzle, verbose=False):
		self.verbose = verbose
		self.puzzle = puzzle
		self.num_cells = len(puzzle.reset())
		self.row_size = int(sqrt(self.num_cells))

	def print(self, msg):
		if self.verbose:
			print(msg)

	def c(self, node):
		cost = 0

		while node is not None:
			cost += node.cost
			node = node.parent

		return cost

	def h(self, node):
		# calculate the manhattan distance
		self.print('\nCalculating h(n) for {}'.format(node))

		score = 0

		for i in range(self.num_cells):
			self.print('Puzzle Piece {}:'.format(i))

			row = int(i / self.row_size)
			col = i % self.row_size
			self.print('\tgoal position is: {}, {}'.format(row, col))

			pos = node.s.index(i)
			curr_row = int(pos/ self.row_size)
			curr_col = pos % self.row_size
			self.print('\tcurrently at: {}, {}'.format(curr_row, curr_col))
			
			dy = abs(curr_row - row)
			dx = abs(curr_col - col)
			distance = dx + dy
			self.print('\tdistance from goal: {} ({}, {})'.format(distance, dx, dy))

			score += distance

		self.print('Heuristic for {}: {}\n'.format(node, score))

		return score


	def solve(self):
		start_time = time.time ()

		root = Node(s=self.puzzle.reset(), action=None, parent=None, cost=0)
		open_set = list()
		closed_set = list()
		open_set.append(root)
		
		while len(open_set) > 0:
			best_score = 2**31

			for node in open_set:
				score = node.cost + self.h(node)
				if score < best_score:
					best_score = score
					i = open_set.index(node)

			n = open_set.pop(i)
			closed_set.append(n.s)

			if self.puzzle.isgoal(n.s):
				print('solution found!')
				terminal_node = n
				break

			for action in self.puzzle.actions(s=n.s):
				next_state = self.puzzle.step(s=n.s, a=action)
				child = Node(s=next_state, action=action, parent=n, cost=n.cost + 1)

				if next_state in closed_set:
					continue

				if child not in open_set:
					open_set.append(child)

		elapsed_time = time.time () - start_time
		print("Elapsed time: {:.2f} seconds".format(elapsed_time))

		if terminal_node != None:
				self.puzzle.show(a=self.action_list(terminal_node))

	def action_list(self, node):
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
	solver = EightPuzzleSolver(puzzle, verbose=args.verbose)
	solver.solve()

if __name__ == '__main__':
	main()