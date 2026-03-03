# Assignment 1 Design Document
## Multi-Robot Path Planning on a Grid

---

### 1. State Representation

The state for each individual robot is a 4-tuple:

```
(x, y, checkpoint_index, t)
```

Here is what each part means:

- **x, y** – the robot's current position on the grid (column and row in the coordinate system where (0,0) is bottom-left).

- **checkpoint_index** – an integer from 0 to K, where K is the total number of checkpoints for this robot. A value of 0 means no checkpoints have been visited yet. A value of 3 (for K=3) means all checkpoints have been visited.

- **t** – the current time step. This is included in the state because two robots are not allowed to occupy the same cell at the same time. Including time in the state lets the planner distinguish between visiting a cell at t=3 versus visiting the same cell at t=7.

**Why this state space?**
Without `checkpoint_index`, there would be no way to know which checkpoints the robot has already hit, so the planner would not know when it is legal to accept the goal. Without `t`, the planner cannot check whether another (higher-priority) robot is using a particular cell at that exact moment.

The total number of possible states per robot is at most:
```
M x N x (K + 1) x (energy_limit + 1)
```
This is always finite, so BFS is guaranteed to terminate.

---

### 2. Search Algorithm: Breadth-First Search (BFS)

I chose **Breadth-First Search (BFS)** as the planning algorithm.

**How it works (briefly):**
BFS explores states level by level, starting from the initial state. All states reachable in 1 step are explored before any state reachable in 2 steps, and so on. Because of this, the first time BFS reaches the goal state, it has done so in the fewest steps possible.

**Why BFS is the right choice here:**
Every action the robot can take (move to a neighbor cell or wait in place) costs exactly 1 unit of energy. This means all step costs are equal – there is no cheaper or more expensive move. When step costs are all equal, BFS finds the optimal (minimum cost) path. Using BFS therefore also minimizes the total energy used, which keeps the robot well within its energy limit.

**How the state transitions work:**
From any state `(x, y, cp_idx, t)`, the planner generates successor states by:
1. Looking at which cells the robot can move to from `(x, y)`, following the grid rules (bounds, obstacles, one-way constraints).
2. Optionally staying in place (wait), unless the robot is currently on a one-way cell (one-way cells force movement).
3. Skipping any destination `(nx, ny)` that is reserved by a higher-priority robot at time `t+1`.
4. If the destination matches the next required checkpoint, incrementing `checkpoint_index`.

The BFS terminates successfully when it dequeues a state where `checkpoint_index == K` and the position equals the goal.

**Path reconstruction:**
Rather than storing the full path in the queue (which uses a lot of memory), the implementation uses a `parent` dictionary that maps each state to the state it came from. Once the goal state is found, the actual path is reconstructed by following the parent chain backwards and then reversing it.

---

### 3. Heuristic

**No heuristic is used.**

BFS is an uninformed (blind) search algorithm and does not use a heuristic function to estimate distance to the goal. Since BFS already finds the shortest path when all step costs are equal, there is no need for a heuristic in this case.

If step costs were different (for example, moving over rough terrain costs more), then a heuristic would be useful for guiding the search more efficiently (like in A*). But that is not the case here.

---

### 4. Priority-Based Multi-Robot Planning

To plan paths for multiple robots without requiring advanced techniques, I used a **sequential priority-based approach**:

1. Sort all robots in descending order of priority (highest priority first).
2. Plan a path for the highest-priority robot without any restrictions from other robots.
3. Record every `(x, y, t)` cell that robot occupies during its path as a **reservation**.
4. Assume the robot stays at its goal position after its path ends, so add reservations for the goal cell at all future time steps too.
5. Plan the next robot, treating all existing reservations as blocked cells at those time steps.
6. Repeat for all remaining robots.

If a robot cannot find any valid path (even with waiting and detours) within its energy limit, an error is reported for that robot.

This approach is straightforward and only relies on single-robot search (BFS), applied repeatedly.

---

### 5. One-Way Cell Handling

When a robot is currently standing on a one-way cell (`^`, `v`, `<`, `>`), it **must** move in the mandated direction on the very next step. Waiting is not allowed. If the forced direction leads out of bounds or into an obstacle, the robot cannot legally be on that one-way cell at all (which the grid is assumed not to have as a start/goal/checkpoint).

Entering a one-way cell is allowed from any direction.

---

### 6. Collision Handling

The assignment only requires avoiding **same-cell collisions** (two robots at the same `(x, y)` at the same time `t`). Edge-swap collisions (two robots swapping positions in one step) are **not** required to be handled and are not handled in this implementation.

---
