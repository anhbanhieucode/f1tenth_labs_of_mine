grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']

def search(grid, init, goal, cost):
    closed = [[0 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    closed[init[0]][init[1]] = 1
    expand = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]
    x = init[0]
    y = init[1]
    g = 0

    open_list = [[g, x, y]]

    found = False
    resign = False

    while not found and not resign:
        if len(open_list) == 0:
            resign = True
            return 'fail'
        else:
            open_list.sort()
            open_list.reverse()
            next_item = open_list.pop()
            x = next_item[1]
            y = next_item[2]
            g = next_item[0]
            expand[x][y] = g

            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open_list.append([g2, x2, y2])
                            closed[x2][y2] = 1

    # Generate the path
    path = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]
    x = goal[0]
    y = goal[1]
    path[x][y] = '*'
    while x != init[0] or y != init[1]:
        for i in range(len(delta)):
            x2 = x - delta[i][0]
            y2 = y - delta[i][1]
            if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]) and closed[x2][y2] == 1 and expand[x2][y2] == expand[x][y] - cost:
                path[x2][y2] = delta_name[i]
                x = x2
                y = y2
                break

    return path,expand

path,expand = search(grid, init, goal, cost)
for row in path:
    print(row)
print(' ')
for close in expand:
    print(close)