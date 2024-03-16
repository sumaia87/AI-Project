import math
import queue
import time

# Constants
INF = 1000000
FREE = 0
BLOCK = 1
PATH = -1
N = 101
row, column = 0, 0
par = [[(0, 0)] * N for _ in range(N)]
Maze = [[0] * N for _ in range(N)]
Manhattan_Distance = [[0] * N for _ in range(N)]
Diagonal_Distance = [[0] * N for _ in range(N)]
Euclidean_Distance = [[0] * N for _ in range(N)]
fx = [0, -1, 0, +1, -1, -1, +1, +1]
fy = [+1, 0, -1, 0, +1, -1, -1, +1]

# Function to check if a cell is valid
def isValid(currentX, currentY):
    return 0 <= currentX < row and 0 <= currentY < column

# Function to check if a cell is unblocked
def isUnBlocked(row, col):
    return Maze[row][col] == FREE

# Function to calculate Manhattan distances
def Manhattan_Distances(goal):
    for currentX in range(row):
        for currentY in range(column):
            Manhattan_Distance[currentX][currentY] = abs(goal[0] - currentX) + abs(goal[1] - currentY)

# Function to calculate Diagonal distances
def Diagonal_Distances(goal):
    for currentX in range(row):
        for currentY in range(column):
            Diagonal_Distance[currentX][currentY] = max(abs(goal[0] - currentX), abs(goal[1] - currentY))

# Function to calculate Euclidean distances
def Euclidean_Distances(goal):
    for currentX in range(row):
        for currentY in range(column):
            Euclidean_Distance[currentX][currentY] = math.sqrt(
                ((goal[0] - currentX) * (goal[0] - currentX)) + ((goal[1] - currentY) * (goal[1] - currentY)))

# Function for A* Search algorithm
def AStarSearch(source, goal, method):
    start_time = time.time()
    if method == 1:
        heuristic = Manhattan_Distance
        print("Method 1: Euclidean g(n) + Manhattan h(n)")
    elif method == 2:
        heuristic = Diagonal_Distance
        print("Method 2: Euclidean g(n) + Diagonal h(n)")
    else:
        print("Invalid method number")
        return

    if not isValid(*source) or not isValid(*goal) or not isUnBlocked(*source) or not isUnBlocked(*goal):
        print("Invalid source or goal")
        return

    if source == goal:
        print("Source is the same as the goal")
        return

    Distance = Euclidean_Distance
    cost = [[INF] * N for _ in range(N)]
    dist = [[INF] * N for _ in range(N)]
    pq = queue.PriorityQueue()
    pq.put((0, source))
    cost[source[0]][source[1]] = 0

    while not pq.empty():
        current_cost, current = pq.get()
        current_x, current_y = current

        for k in range(4):
            nxt_x = current_x + fx[k]
            nxt_y = current_y + fy[k]
            if isValid(nxt_x, nxt_y) and Maze[nxt_x][nxt_y] != BLOCK and \
                    current_cost + Distance[current_x][current_y] + 1 < cost[nxt_x][nxt_y]:
                dist[nxt_x][nxt_y] = dist[current_x][current_y] + 1
                cost[nxt_x][nxt_y] = current_cost + Distance[current_x][current_y] + 1
                par[nxt_x][nxt_y] = current
                pq.put((cost[nxt_x][nxt_y], (nxt_x, nxt_y)))

        for k in range(4, 8):
            nxt_x = current_x + fx[k]
            nxt_y = current_y + fy[k]
            if isValid(nxt_x, nxt_y) and Maze[nxt_x][nxt_y] != BLOCK and \
                    current_cost + Distance[current_x][current_y] + 1.414 < cost[nxt_x][nxt_y]:
                dist[nxt_x][nxt_y] = dist[current_x][current_y] + 1
                cost[nxt_x][nxt_y] = current_cost + Distance[current_x][current_y] + 1.414
                par[nxt_x][nxt_y] = current
                pq.put((cost[nxt_x][nxt_y], (nxt_x, nxt_y)))

    if dist[goal[0]][goal[1]] == INF:
        print("Cannot reach goal")
        return

    path = []
    cur = goal
    while cur != source:
        path.append(cur)
        cur = par[cur[0]][cur[1]]
    path.append(source)
    path.reverse()

    print("Path:")
    for p in path:
        print(f"({p[0]}, {p[1]})", end=" -> ")
    print(f"({goal[0]}, {goal[1]})")

    pathcost = 0
    for i in range(1, len(path)):
        if (path[i][0] - path[i - 1][0]) ** 2 + (path[i][1] - path[i - 1][1]) ** 2 == 1:
            pathcost += 1
        else:
            pathcost += 1.414

    print("Path Cost:", pathcost)
    print("Execution Time:", round(time.time() - start_time, 4), "seconds")
    print("\n")

if __name__ == "__main__":
    row, column = map(int, input("Enter maze dimensions (row column): ").split())

    for x in range(row):
        for y in range(column):
            Maze[x][y] = FREE

    obstacles = int(input("Enter number of obstacles: "))

    for _ in range(obstacles):
        x, y = map(int, input("Enter obstacle coordinates (x y): ").split())
        Maze[x][y] = BLOCK

    source = tuple(map(int, input("Enter source coordinates (x y): ").split()))
    goal = tuple(map(int, input("Enter goal coordinates (x y): ").split()))

    Manhattan_Distances(goal)
    Diagonal_Distances(goal)
    Euclidean_Distances(goal)

    AStarSearch(source, goal, 1)
    AStarSearch(source, goal, 2)
