import numpy as np
import matplotlib.pyplot as plt
import random

from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22

# treeNode class
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None

class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.iterations = min(numIterations, 200)
        self.grid = grid
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []

    # add the point to the nearest node and add goal when reached
    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX, locationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode

    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        #print(point)
        return point

    # steer a distance stepsize from start to end location
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho * self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1] - 1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0] - 1
        return point

    # check if there is an obstacle
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = locationStart.locationX + i * u_hat[0]
            testPoint[1] = locationStart.locationY + i * u_hat[1]
            if self.grid[round(testPoint[1]).astype(np.int64), round(testPoint[0]).astype(np.int64)] == 1:
                return True
        return False

    # find unit vector between 2 points which form a vector
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v / np.linalg.norm(v)
        #print(u_hat)
        return u_hat

    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        #dist = self.distance(root, [point.locationX, point.locationY])
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)
            #print(self.nearestDist)
        pass

    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)
        return dist

    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        pass

    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    def retraceRRTPath(self, goal):
        if goal.locationX == self.randomTree.locationX:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0, currentPoint)
        self.path_distance += self.rho
        self.retraceRRTPath(goal.parent)

grid = np.load('/home/anhbanhieu/catkin_ws/scpace.npy')
start = np.array([100.0, 100.0])
goal = np.array([1050.0, 450.0])
numIterations = 400
stepSize = 50
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill=False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

rrt = RRTAlgorithm(start, goal, numIterations, grid, stepSize)
for i in range(rrt.iterations):
    rrt.resetNearestValues()
    print("Iteration:", i)
    #print(rrt.randomTree)
    point = rrt.sampleAPoint()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool_val = rrt.isInObstacle(rrt.nearestNode, new)
    if bool_val == False:
        rrt.addChild(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle="--")
        if rrt.goalFound(new):
            rrt.addChild(goal[0], goal[1])
            print("Goal found!")
            break

rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0, start)
print("Number of waypoints:", rrt.numWaypoints)
print("Path Distance (m):", rrt.path_distance)
print("Waypoints:", rrt.Waypoints)

# plot waypoints
for i in range(len(rrt.Waypoints) - 1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i + 1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i + 1][1]], 'ro',
             linestyle="--")
    plt.pause(0.10)

plt.show()
