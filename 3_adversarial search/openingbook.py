import random
import pickle
from collections import defaultdict, Counter
from isolation import *
from isolation.isolation import Action, _ACTIONSET
from sample_players import *
import time
import  math

horizontal_actions = {Action.NNE:Action.NNW,Action.ENE:Action.WNW,Action.ESE:Action.WSW,Action.SSE:Action.SSW,Action.SSW:Action.SSE,Action.WSW:Action.ESE,Action.WNW:Action.ENE,Action.NNW:Action.NNE}
vertical_actions = {Action.NNE:Action.SSE,Action.ENE:Action.ESE,Action.ESE:Action.ENE,Action.SSE:Action.NNE,Action.SSW:Action.NNW,Action.WSW:Action.WNW,Action.WNW:Action.WSW,Action.NNW:Action.SSW}
diagonal_actions = {Action.NNE:Action.SSW,Action.ENE:Action.WSW,Action.ESE:Action.WNW,Action.SSE:Action.NNW,Action.SSW:Action.NNE,Action.WSW:Action.ENE,Action.WNW:Action.ESE,Action.NNW:Action.SSE}


def alpha_beta_search(state, depth=5):

    def min_value(state, depth, alpha, beta):
        if state.terminal_test():
            return state.utility(state.player())
        if depth <= 0:
            return score(state)

        v = float("inf")

        for a in state.actions():
            v = min(v, max_value(state.result(a), depth-1, alpha, beta))
            if v <= alpha:
                return v
            beta = min(beta, v)
        return v

    def max_value(state, depth, alpha, beta):
        if state.terminal_test():
            return state.utility(state.player())
        if depth <= 0:
            return score(state)
        v = float("-inf")
        for a in state.actions():
            v = max(v, min_value(state.result(a), depth-1, alpha, beta))
            if v >= beta:
                return v
            alpha = max(alpha, v)
        return v

    alpha = float("-inf")
    beta = float("inf")
    best_score = float("-inf")
    best_move = None
    for a in state.actions():
        v = min_value(state.result(a), depth-1, alpha, beta)
        alpha = max(alpha, v)
        if v >= best_score:
            best_score = v
            best_move = a
    return best_move


def score(state):
    own_loc = state.locs[state.player()]
    opp_loc = state.locs[1 - state.player()]
    mid = 57

    w = 11
    own_x, own_y = own_loc % (w + 2), own_loc // (w + 2)
    opp_x, opp_y = opp_loc % (w + 2), opp_loc // (w + 2)
    mid_x, mid_y = mid % (w + 2), mid // (w + 2)
    dist = math.sqrt(((own_x - mid_x) ** 2) + ((own_y - mid_y) ** 2))
    dist2 = math.sqrt(((opp_x - mid_x) ** 2) + ((opp_y - mid_y) ** 2))

    own_liberties = state.liberties(own_loc)
    opp_liberties = state.liberties(opp_loc)
    return (len(own_liberties) - dist) - (len(opp_liberties) - dist2)

def rotate_state_action(state, action):
    w = 11
    h = 9
    board = bin(state.board)[2:]
    board = [board[i*(w+2):(i+1)*(w+2)] for i in range(h)]

    own_loc = state.locs[0]
    opp_loc = state.locs[1]
    row1, col1 = (own_loc // (w + 2) if own_loc != None else None, own_loc % (w + 2) if own_loc != None else None)
    row2, col2 = (opp_loc // (w + 2) if opp_loc != None else None, opp_loc % (w + 2) if opp_loc != None else None)

    board_horizontal = eval('0b'+''.join([row[::-1] for row in board]))
    players_horizontal = (row1 * (w+2) + (w-1-col1) if own_loc != None else None,
                          row2 * (w+2) + (w-1-col2) if opp_loc != None else None)
    state_horizontal = Isolation(board=board_horizontal, ply_count=state.ply_count, locs=players_horizontal)
    if action not in _ACTIONSET:
        row3, col3 = action // (w + 2), action % (w + 2)
        action_horizontal = row3 * (w+2) + (w-1-col3)
    else:
        action_horizontal = horizontal_actions[action]

    board_vertical = eval('0b'+''.join(board[::-1]))
    players_vertical = ((h-1-row1) * (w+2) + col1 if own_loc != None else None,
                        (h-1-row2) * (w+2) + col2 if opp_loc != None else None)
    state_vertical = Isolation(board=board_vertical, ply_count=state.ply_count, locs=players_vertical)
    if action not in _ACTIONSET:
        row3, col3 = action // (w + 2), action % (w + 2)
        action_vertical = (h-1-row3) * (w+2) + col3
    else:
        action_vertical = vertical_actions[action]

    board_diagonal = eval('0b'+''.join([row[::-1] for row in board[::-1]]))
    players_diagonal = ((h-1-row1) * (w+2) + (w-1-col1) if own_loc != None else None,
                        (h-1-row2) * (w+2) + (w-1-col2) if opp_loc != None else None)
    state_diagonal = Isolation(board=board_diagonal, ply_count=state.ply_count, locs=players_diagonal)
    if action not in _ACTIONSET:
        row3, col3 = action // (w + 2), action % (w + 2)
        action_diagonal = (h-1-row3) * (w+2) + (w-1-col3)
    else:
        action_diagonal = diagonal_actions[action]

    return [(state_horizontal,action_horizontal),(state_vertical,action_vertical),(state_diagonal,action_diagonal)]



NUM_ROUNDS = 1000

def build_table(num_rounds=NUM_ROUNDS):

    book = defaultdict(Counter)
    for i in range(num_rounds):
        state = Isolation()
        build_tree(state, book)
    return {k: max(v, key=v.get) for k, v in book.items()}


def build_tree(state, book, depth=5):
    if depth <= 0 or state.terminal_test():
        return -simulate(state)

    action = alpha_beta_search(state=state, depth=5)
    reward = build_tree(state.result(action), book, depth - 1)

    rotations = rotate_state_action(state, action)
    for i, item in enumerate(rotations):
        book[item[0]][item[1]] += reward
    book[state][action] += reward
    return -reward


def simulate(state):
    while not state.terminal_test():
        state = state.result(random.choice(state.actions()))
    return -1 if state.utility(state.player()) < 0 else 1


if __name__ == "__main__":
    open_book = build_table(NUM_ROUNDS)
    with open("data.pickle", 'wb') as f:
        pickle.dump(open_book, f)