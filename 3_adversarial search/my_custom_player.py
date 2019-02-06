
from sample_players import DataPlayer


class CustomPlayer(DataPlayer):

    def get_action(self, state):

        import random
        if state.ply_count < 2:
            self.queue.put(random.choice(state.actions()))
        else:
            self.queue.put(self.alpha_beta_search(state, depth=5))

    def alpha_beta_search(self, state, depth):

        def min_value(state, depth, alpha, beta):
            if state.terminal_test():
                return state.utility(self.player_id)
            if depth <= 0:
                return self.score(state)

            v = float("inf")

            for a in state.actions():
                v = min(v, max_value(state.result(a), depth-1, alpha, beta))
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


    def score(self, state):
        own_loc = state.locs[self.player_id]
        opp_loc = state.locs[1 - self.player_id]
        own_liberties = state.liberties(own_loc)
        opp_liberties = state.liberties(opp_loc)
        return len(own_liberties) - len(opp_liberties)