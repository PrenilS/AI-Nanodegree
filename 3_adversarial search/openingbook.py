import random
import pickle
from collections import defaultdict, Counter
from isolation import *
from sample_players import *
import time


def alpha_beta_search(state, depth=3):

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
    own_liberties = state.liberties(own_loc)
    opp_liberties = state.liberties(opp_loc)
    return len(own_liberties) - len(opp_liberties)




NUM_ROUNDS = 100

def build_table(num_rounds=NUM_ROUNDS):

    book = defaultdict(Counter)
    for i in range(num_rounds):
        state = Isolation()
        build_tree(state, book)
    return {k: max(v, key=v.get) for k, v in book.items()}


def build_tree(state, book, depth=4):
    if depth <= 0 or state.terminal_test():
        return -simulate(state)

    action = alpha_beta_search(state=state, depth=3)
    reward = build_tree(state.result(action), book, depth - 1)
    book[state][action] += reward
    return -reward


def simulate(state):
    while not state.terminal_test():
        state = state.result(random.choice(state.actions()))
    return -1 if state.utility(state.player()) < 0 else 1


if __name__ == "__main__":
    print("Enter>>>")
    start = time.time()
    open_book = build_table(NUM_ROUNDS)
    print(open_book)
    end = time.time()
    print("The total time for {} rounds is {} seconds, the average time for each round is {} seconds per round".format(NUM_ROUNDS,end-start,(end-start)/NUM_ROUNDS))
    with open("data.pickle", 'wb') as f:
        pickle.dump(open_book, f)