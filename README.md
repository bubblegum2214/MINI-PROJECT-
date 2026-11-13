Mini-Project 1: Search Algorithms for a 
Cleaning Robot 
Course: CS 5550 – Introduction to Artificial Intelligence 
Project Description 
This project explores three search algorithms: Depth-First Search (DFS), A* Search, and 
Iterative Deepening A* (IDA*). 
The algorithms are implemented in a grid-based vacuum-cleaning environment. The 
objective is to find a sequence of actions that cleans all dirty cells while minimizing the total 
path cost. 
Algorithms 
 DFS: Explores states using a stack. 
 A:* Uses path costs and heuristics to find optimal solutions. 
 IDA:* Uses iterative deepening with heuristic cost thresholds. 
Heuristics 
 h0: Zero heuristic. 
 h1: Number of remaining dirty cells. 
 h2: Number of remaining dirty cells plus Manhattan distance to the nearest dirty cell. 
Project Files 
 vacuum_world.py – Vacuum-world environment. 
 search.py – Search algorithms. 
 heuristics.py – Heuristic functions. 
 experiments.py – Experimental comparisons. 
 priority_queue.py – Priority queue helper. 
 test_grids.py – Test environments. 
 run_tests.py – Project tests. 
 results.csv – Experimental results. 
 figures/ – Performance graphs. 
 report.pdf – Final project report. 
 AI_USE.md – AI assistance disclosure. 
Requirements 
 Python 3 
 pandas 
 matplotlib 
Install the required packages: 
python -m pip install pandas matplotlib 
How to Run 
Open a terminal in the project folder. 
Run the tests: 
python run_tests.py 
Run the experiments: 
python experiments.py 
The experimental results are stored in results.csv, and the generated plots are saved in the 
figures folder. 
Experimental Results 
The experiments compare solution cost, nodes expanded, frontier size or iterations, and 
execution time across six grid configurations. 
The report contains the experimental tables, graphs, heuristic analysis, discussion, and 
reflection.
