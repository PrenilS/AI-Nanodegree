# Udacity AI Nanodegree Project 2 on Classical Planning

This repository constains code which creates a forward planning agent in order to solve 4 different air cargo problems.

The problems increase in complexity from 1 to 4 and all involve moving specific peices of cargo to specific airports with a fixed number of planes.

Both informed and uninformed search algorithms are tested on the problems.

The idea is to gain an understanding of which algorithms perform best at different levels of complexity.

You can run the problems yourself from shell:

```
python run_search.py -p 1 2 3 4 -s 1 2 3 4 5 6 7 8 9 10 11
```

The `-p` option allows you to select which of the 4 problems and the `-s` option allows you to select which algorithm to use.
The algorithms are as follows:

1. Breath first search
2. Depth first graph search
3. Uniform cost search
4. Greedy best first graph search, unmet goals heuristic
5. Greedy best first graph search, level sum heuristic
6. Greedy best first graph search, max level heuristic
7. Greedy best first graph search, set level heuristic
8. A* search, unmet goals heuristic
9. A* search, level sum heuristic
10. A* search, max level heuristic
11. A* search, set level heuristic

Alternatively, `python run_search.py -m`, will bring up messages to help you manually choose which problems and algorithms to use.

report.pdf contains my own conclusions from this exercise and a few graphs illustrating the performances of each algorithm.
