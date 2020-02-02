from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import helper
import numpy as np


def for_loop():
    # MISSING: Write a for-loop that builds an array containing the
    # cubic values of 5, 6, 7, 8, 9 and 10.
    # BONUS: Can you use list comprehensions instead?
    return [x**3 for x in range(5, 11)]


def format_string(v):
    # MISSING: Return a string displaying the number v with 2 decimals
    # places. If v = 1.2345, output is '1.23'.
    return '%.2f' % v


def filter_list(a, pred):
    # MISSING: Return a list that contains only elements v of a for
    # which pred(v) returns True.
    # E.g.: filter_list(range(4), lambda v: return v % 2 == 0)
    #       returns [0, 2].
    return list(filter(pred, a))


def find_root(fn):
    # MISSING: Returns the value x for which fn(x) ~= 0 using the bisection method.
    # x should be correct within 1e-3.
    # fn is strictly monotonic and is always such that fn(0.) < 0 and fn(1.) > 0.
    start = 0.
    end = 1.
    diff = end-start
    mp = (start + end) / 2.
    while diff > 1e-3:
        mp = (start + end) / 2.
        if fn(mp) > 0:
            end = mp
        else:
            start = mp
        diff = abs(start-end)
    return mp


def finished_tictactoe(state):
    # MISSING: state is a list of strings. Each string contains 3 characters.
    # Each character can be 'X', 'O', or ' '. Return whether the Tic Tac Toe
    # position represented by this list is an ending position (no one else can
    # play).
    # E.g.: state = ['XO ']
    #                ' X ',
    #                'O X'] returns True.
    # E.g.: state = ['XO ']
    #                ' OX',
    #                'O X'] returns False.
    # BONUS: Can you think of a concise way of writing this function using NumPy?
    a = np.array([list(s) for s in state])
    x_wins = a == 'X'
    o_wins = a == 'O'
    x = np.all(x_wins, axis=0).any() or np.all(x_wins, axis=1).any(
    ) or np.diag(x_wins).all() or np.diag(x_wins[:, ::-1]).all()
    o = np.all(o_wins, axis=0).any() or np.all(o_wins, axis=1).any(
    ) or np.diag(o_wins).all() or np.diag(o_wins[:, ::-1]).all()
    return x or o


def main():
    print('Welcome to the Mobile Robots course.\n')

    helper.check_solution(for_loop, [125, 216, 343, 512, 729, 1000])

    helper.check_solution(format_string, '1.23', 1.2345)
    helper.check_solution(format_string, '3.14', math.pi)

    helper.check_solution(filter_list, [1, 3], range(5), lambda v: v % 2 == 1)
    helper.check_solution(filter_list, [], range(5), lambda v: False)

    helper.check_solution(find_root, 0.5, lambda x: x * 2 - 1., approx=True)
    helper.check_solution(
        find_root, 0.7071, lambda x: x ** 2 - .5, approx=True)

    helper.check_solution(finished_tictactoe, True, ['XO ', ' X ', 'O X'])
    helper.check_solution(finished_tictactoe, True, [' O ', 'XXX', 'O  '])
    helper.check_solution(finished_tictactoe, True, ['XO ', 'X  ', 'X O'])
    helper.check_solution(finished_tictactoe, False, ['XO ', ' OX', 'O X'])
    helper.check_solution(finished_tictactoe, True, ['XOO', 'XOX', 'OXX'])


if __name__ == '__main__':
    main()
