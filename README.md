# Multi-Robot Path Planning on a Grid

Assignment 1 for the AI/search algorithms course. The goal is to plan collision-free paths for multiple robots moving on a 2D grid, where each robot has a start, a goal, and an ordered list of checkpoints it must visit along the way.

---

## Problem Overview

The grid is made up of cells that can be:
- `.` free cell (robot can enter from any direction)
- `X` obstacle (robot can never enter)
- `^` `v` `<` `>` one-way cells (robot can enter freely, but when leaving it must go in the marked direction)

Each robot has:
- A unique ID and a priority
- An energy limit (maximum number of moves it can make)
- A start position, a goal position, and zero or more checkpoints to visit in order

Robots are planned one at a time from highest to lowest priority. Each robot's planned path is recorded as time-stamped cell reservations, and all later robots must route around those reservations to avoid collisions.

---

## Algorithm

The planner uses **Breadth-First Search (BFS)**.

The state for a single robot is `(x, y, checkpoint_index, t)`:
- `x, y` is the robot's current position
- `checkpoint_index` tracks how many checkpoints have been visited so far
- `t` is the current time step, used to check collision with other robots' reservations

Since every action (move or wait) costs exactly 1 unit of energy, all step costs are equal and BFS finds the minimum-energy path. No heuristic is used.

More detail on the design choices is in [design_document.md](design_document.md).

---

## Files

| File | Description |
|------|-------------|
| `assignment1.py` | Main solution script |
| `input.txt` | Input file the script reads from |
| `output.txt` | Output file the script writes to (generated on run) |
| `design_document.md` | Written explanation of state space, algorithm, and heuristic choices |

---

## How to Run

Make sure you have Python 3 installed. No external libraries are needed.

1. Put your grid and robot definitions in `input.txt` (see format below).
2. Run the script:

```bash
python assignment1.py
```

3. Check `output.txt` for the results.

---

## Input Format

```
N M
<row 1 of grid, top row, y = N-1>
<row 2>
...
<row N, bottom row, y = 0>
R
<for each robot:>
ID Priority EnergyLimit
startX startY
goalX goalY
K
cpX cpY   (K lines, one checkpoint per line)
```

Coordinates use `(0, 0)` at the bottom-left corner. `x` increases to the right, `y` increases upward.

---

## Output Format

For each robot, sorted by ascending Robot ID:

```
Robot <ID>:
Path: (x0,y0) -> (x1,y1) -> ... -> (xT,yT)
Total Time: T
Total Energy: E
```

If no valid path is found for a robot:

```
Error: No valid path found for Robot <ID>
```

---

## Example

The included `input.txt` is the example from the assignment:

```
5 5
.....
.X.X.
.....
.X.X.
.....
2
1 2 30
0 0
4 4
1
2 2
2 1 25
4 0
0 4
1
2 2
```

Running `python assignment1.py` produces output like:

```
Robot 1:
Path: (0,0) -> (0,1) -> (0,2) -> (1,2) -> (2,2) -> (2,3) -> (2,4) -> (3,4) -> (4,4)
Total Time: 8
Total Energy: 8

Robot 2:
Path: (4,0) -> (4,1) -> (4,2) -> (3,2) -> (3,2) -> (2,2) -> (2,3) -> (2,4) -> (1,4) -> (0,4)
Total Time: 9
Total Energy: 9
```

Robot 1 (higher priority) is planned first. Its path goes bottom-left to top-right via checkpoint (2,2) in 8 moves. Robot 2 is planned second, avoids the cells Robot 1 uses at each time step, and waits one step at (3,2) so it can pass through (2,2) after Robot 1 has moved on.
