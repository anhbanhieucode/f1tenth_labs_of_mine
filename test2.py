def compute_value(grid, goal, cost):
    value = [[99 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    value[goal[0]][goal[1]] = 0

    change = True
    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if goal[0] == x and goal[1] == y:
                    continue

                if grid[x][y] == 0:
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]

                        if (
                            x2 >= 0
                            and x2 < len(grid)
                            and y2 >= 0
                            and y2 < len(grid[0])
                            and grid[x2][y2] == 0
                        ):
                            v2 = value[x2][y2] + cost

                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2

        # Print the value grid after each iteration
        print (v2)
        for row in value:
            print(row)
        print("--------")  # Separate iterations

    return value

# Example usage
grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

value_grid = compute_value(grid, goal, cost)

# Print the final value grid
for row in value_grid:
    print(row)
