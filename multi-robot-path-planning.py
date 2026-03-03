from collections import deque


# read input file
def read_input(filename='input.txt'):
    """
    Input file format:
        Line 1      : N M  (grid height and width)
        Lines 2..N+1: grid rows, top row first (top row = y coordinate N-1)
        Line N+2    : R  (number of robots)
        Then for each robot:
            ID Priority EnergyLimit
            startX startY
            goalX  goalY
            K  (number of checkpoints)
            cpX cpY   (checkpoint coordinates, one line per checkpoint)

    Returns:
        N       (int)  - number of rows
        M       (int)  - number of columns
        grid    (list) - list of N strings; grid[0] is the top row (y = N-1)
        robots  (list) - list of dicts, one per robot
    """
    with open(filename, 'r') as f:
        lines = [line.rstrip('\n') for line in f]

    idx = 0  # current line index while reading

    # --- Grid dimensions ---
    N, M = map(int, lines[idx].split())
    idx += 1

    
    grid = []
    for _ in range(N):
        grid.append(lines[idx])
        idx += 1

    # no of robots
    R = int(lines[idx])
    idx += 1

    # robot attributes
    robots = []
    for _ in range(R):
        
        robot_id, priority, energy_limit = map(int, lines[idx].split())
        idx += 1

        start_x, start_y = map(int, lines[idx].split())
        idx += 1

        goal_x, goal_y = map(int, lines[idx].split())
        idx += 1

        # of checkpoints
        K = int(lines[idx])
        idx += 1

        checkpoints = []
        for _ in range(K):
            cp_x, cp_y = map(int, lines[idx].split())
            checkpoints.append((cp_x, cp_y))
            idx += 1

        robots.append({
            'id':           robot_id,
            'priority':     priority,
            'energy_limit': energy_limit,
            'start':        (start_x, start_y),
            'goal':         (goal_x, goal_y),
            'checkpoints':  checkpoints
        })

    return N, M, grid, robots


# Helper Functions for Grid Navigation
def get_cell(grid, N, x, y):
    row_index = N - 1 - y
    return grid[row_index][x]


def get_successors(grid, N, M, x, y):
    
    cell = get_cell(grid, N, x, y)

    # first check one way cell rules
    ONE_WAY_DIRECTIONS = {
        '^': ( 0, +1),  # up
        'v': ( 0, -1),  # down
        '<': (-1,  0),  # left
        '>': (+1,  0),  # right
    }

    candidates = []
    can_wait = True

    if cell in ONE_WAY_DIRECTIONS:
        # move in the specified direction
        dx, dy = ONE_WAY_DIRECTIONS[cell]
        candidates = [(x + dx, y + dy)]
        can_wait = False
    else:
        # move in all four directions
        candidates = [
            (x,     y + 1),  # up
            (x,     y - 1),  # down
            (x - 1, y    ),  # left
            (x + 1, y    ),  # right
        ]
        can_wait = True

    # handle out of bounds and obstacles
    valid_moves = []
    for (nx, ny) in candidates:
        if 0 <= nx < M and 0 <= ny < N:      # within grid bounds
            if get_cell(grid, N, nx, ny) != 'X':  # not an obstacle
                valid_moves.append((nx, ny))

    return valid_moves, can_wait



# BFS for single robot pathfinding with time and checkpoint tracking
def bfs_plan(robot, grid, N, M, reservations):
    start_x, start_y = robot['start']
    goal_x, goal_y = robot['goal']
    checkpoints = robot['checkpoints']
    energy_limit = robot['energy_limit']
    K = len(checkpoints)

    start_state = (start_x, start_y, 0, 0)

    queue = deque([start_state])

    # We need to track visited states to avoid cycles
    parent = {start_state: None}
    goal_state = None  

    while queue:
        state = queue.popleft()
        x, y, cp_idx, t = state

        # The robot must be at the goal position AND have visited all checkpoints
        if cp_idx == K and x == goal_x and y == goal_y:
            goal_state = state
            break  # BFS found the shortest path; stop searching

        # if exhausted energy limit, cannot continue from this state
        if t >= energy_limit:
            continue

        # Get valid next positions based on the current cell's rules
        valid_moves, can_wait = get_successors(grid, N, M, x, y)

        # wait or move to next positions
        next_positions = list(valid_moves)
        if can_wait:
            next_positions.append((x, y))  # waiting = stay at the same cell

        for (nx, ny) in next_positions:
            next_t = t + 1

            # skip if next position at next time step is reserved by a higher-priority robot
            if (nx, ny, next_t) in reservations:
                continue

            # If the robot moves to the next required checkpoint, advance the index
            next_cp_idx = cp_idx
            if cp_idx < K and (nx, ny) == checkpoints[cp_idx]:
                next_cp_idx = cp_idx + 1

            next_state = (nx, ny, next_cp_idx, next_t)

            # Only explore states we haven't seen before
            if next_state not in parent:
                parent[next_state] = state  # record where we came from
                queue.append(next_state)

    # If we never reached the goal, return None to indicate failure
    if goal_state is None:
        return None

    # Reconstruct the path by walking backwards through the parent chain
    path = []
    current = goal_state
    while current is not None:
        x, y, _, _ = current
        path.append((x, y))
        current = parent[current]
    path.reverse()  # path was built backwards; flip it to get start -> goal order

    return path


# Priority-based multi-robot path planning
def plan_all_robots(robots, grid, N, M):
    # descending priority order (higher priority first)
    sorted_robots = sorted(robots, key=lambda r: r['priority'], reverse=True)

    max_energy = max(r['energy_limit'] for r in robots)

    # Accumulated set of (x, y, t) cells that are already claimed
    reservations = set()

    results = {}  # robot_id -> path or None

    for robot in sorted_robots:
        path = bfs_plan(robot, grid, N, M, reservations)
        results[robot['id']] = path

        if path is not None:
            T = len(path) - 1  # number of moves made

            # Reserve every cell the robot occupies at each time step
            for t, (px, py) in enumerate(path):
                reservations.add((px, py, t))

            # reserve the goal cell for all time steps after the robot arrives, to prevent others from moving into it
            goal_x, goal_y = path[-1]
            for t in range(T + 1, max_energy + 1):
                reservations.add((goal_x, goal_y, t))

    return results


# Output formatting
def write_output(robots, results, filename='output.txt'):
    """
    OUTPUT FORMAT:
    For each robot sorted by ascending Robot ID:
        If a path was found:
            Robot <ID>:
            Path: (x0,y0) -> (x1,y1) -> ... -> (xT,yT)
            Total Time: T
            Total Energy: E

        If no path was found:
            Error: No valid path found for Robot <ID>

    A blank line is written between consecutive robot outputs.

    Args:
        robots   (list) - all robot definitions (used to get IDs in order)
        results  (dict) - mapping robot_id -> path or None
        filename (str)  - output file path
    """
    # Output robots in order of increasing Robot ID (as required)
    sorted_robots = sorted(robots, key=lambda r: r['id'])

    output_lines = []

    for i, robot in enumerate(sorted_robots):
        rid  = robot['id']
        path = results.get(rid)

        if path is None:
            output_lines.append(f"Error: No valid path found for Robot {rid}")
        else:
            T = len(path) - 1  # number of moves (path has T+1 positions)
            E = T              # each move or wait costs 1 energy unit

            path_str = ' -> '.join(f'({x},{y})' for x, y in path)

            output_lines.append(f"Robot {rid}:")
            output_lines.append(f"Path: {path_str}")
            output_lines.append(f"Total Time: {T}")
            output_lines.append(f"Total Energy: {E}")

        # add a blank line
        if i < len(sorted_robots) - 1:
            output_lines.append('')

    with open(filename, 'w') as f:
        f.write('\n'.join(output_lines) + '\n')


# MAIN FUNCTION
def main():
    
    print("Reading input from input.txt ...")
    N, M, grid, robots = read_input('input.txt')
    print(f"Grid size: {N} rows x {M} cols | Number of robots: {len(robots)}")

    print("Planning paths: ")
    results = plan_all_robots(robots, grid, N, M)

    print("Writing results to output.txt: ")
    write_output(robots, results)

    # Quick result summary printed to terminal
    print("\nSummary: ")
    for robot in sorted(robots, key=lambda r: r['id']):
        rid  = robot['id']
        path = results[rid]
        if path is not None:
            print(f"  Robot {rid}: path found, {len(path) - 1} move(s)")
        else:
            print(f"  Robot {rid}: no valid path found")

    print("\nDone. See output.txt for full results.")


if __name__ == '__main__':
    main()
