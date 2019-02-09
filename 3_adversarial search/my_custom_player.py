import random
from sample_players import DataPlayer
import math
from isolation.isolation import _WIDTH


class CustomPlayer(DataPlayer):

    def get_action(self, state):

        if state.ply_count < 3:
            self.queue.put(random.choice(state.actions()))
            # if state in self.data:
            # self.queue.put(self.data[state])
        # else:
        # self.queue.put(random.choice(state.actions()))
        else:
            depth_limit = 5
            for depth in range(1, depth_limit + 1):
                best_move = self.alpha_beta_search(state, depth)
            self.queue.put(best_move)

    def alpha_beta_search(self, state, depth):

        def min_value(state, depth, alpha, beta):
            if state.terminal_test():
                return state.utility(self.player_id)
            if depth <= 0:
                return self.score(state)

            v = float("inf")

            for a in state.actions():
                v = min(v, max_value(state.result(a), depth - 1, alpha, beta))
                if v <= alpha:
                    return v
                beta = min(beta, v)
            return v

        def max_value(state, depth, alpha, beta):
            if state.terminal_test():
                return state.utility(self.player_id)
            if depth <= 0:
                return self.score(state)
            v = float("-inf")
            for a in state.actions():
                v = max(v, min_value(state.result(a), depth - 1, alpha, beta))
                if v >= beta:
                    return v
                alpha = max(alpha, v)
            return v

        alpha = float("-inf")
        beta = float("inf")
        best_score = float("-inf")
        best_move = None
        for a in state.actions():
            v = min_value(state.result(a), depth - 1, alpha, beta)
            alpha = max(alpha, v)
            if v >= best_score:
                best_score = v
                best_move = a
        return best_move

    def score(self, state):
        own_loc = state.locs[self.player_id]
        opp_loc = state.locs[1 - self.player_id]
        mid = 57

        WIDTH = 11
        own_x, own_y = own_loc % (WIDTH + 2), own_loc // (WIDTH + 2)
        opp_x, opp_y = opp_loc % (WIDTH + 2), opp_loc // (WIDTH + 2)
        mid_x, mid_y = mid % (WIDTH + 2), mid // (WIDTH + 2)
        dist = math.sqrt(((own_x - mid_x) ** 2) + ((own_y - mid_y) ** 2))
        dist2 = math.sqrt(((opp_x - mid_x) ** 2) + ((opp_y - mid_y) ** 2))

        wght = 0.5
        # wght=1
        # wght=2
        # wght=3

        own_liberties = state.liberties(own_loc)
        opp_liberties = state.liberties(opp_loc)
        return (len(own_liberties) - dist * wght) - (len(opp_liberties) - dist2 * wght)