#RRT Star algorithm
import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 15

#tree Node class
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
        
#RRT Star Algorithm class
class RRTStarAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])          #The RRT (root position) (has 0 cost)
        self.goal = treeNode(goal[0], goal[1])                  #goal position (initialize to a high cost)
        self.nearestNode = None                                 #nearest node            
        self.iterations = min(numIterations, 700)               #number of iterations to run
        self.grid = grid                                        #the map
        self.rho = stepSize                                     #length of each branch   
        self.nearestDist = 10000                                #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                   #number of waypoints
        self.Waypoints = []                                     #the waypoints
        self.searchRadius = self.rho*2                          #the radius to search for finding neighbouring vertices 
        self.neighbouringNodes = []                             #neighbouring nodes  
        self.goalArray = np.array([goal[0],goal[1]])            #goal as an array
        self.goalCosts = [10000]                                #the costs to the goal (ignore first value)
            
    #add the node to the nearest node, and add goal if necessary     
    def addChild(self, treeNode):
        if (treeNode.locationX == self.goal.locationX):
            #append goal to nearestNode's children
            #and set goal's parent to nearestNode
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:    
            #append this node to nearestNode's children
            #set the parent to nearestNode
            self.nearestNode.children.append(treeNode)
            treeNode.parent = self.nearestNode
            
    #sample random point within grid limits (DONE)
    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point
    
    #steer a distance stepSize from start location to end location (keep in mind the grid limits) (DONE)
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1]-1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0]-1
        return point
    
    #check if obstacle lies between the start and end point of the edge (DONE)
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = min(grid.shape[1]-1, locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(grid.shape[0]-1, locationStart.locationY + i*u_hat[1])
            #if self.grid[round(testPoint[1]).astype(np.int64),round(testPoint[0]).astype(np.int64)] == 1:
            if self.grid[round(testPoint[1]),round(testPoint[0])] == 1:
                return True
        return False

    #find the unit vector between 2 locations (DONE)
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    #find the nearest node from a given (unconnected) point (Euclidean distance) 
    def findNearest(self, root, point):
        if not root:
            return
        #find distance between root and point use distance method,
        #if it's lower than or equal to nearestDist then
            #update nearestNode to root
            #update nearestDist to the distance from line 84
        distHere = self.distance(root,point)
        if distHere <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = distHere
        #recursive call
        for child in root.children:
            self.findNearest(child, point)
            
    #find neighbouring nodes        
    def findNeighbouringNodes(self,root,point):
        if not root:
            return
        #find distance between root and point (dist)
        #add root to neighbouringNodes if dist is less than or equal to searchRadius
        distHere2 = self.distance(root,point)
        if distHere2 <= self.searchRadius:
            self.neighbouringNodes.append(root)
        #recursive call
        for child in root.children:
            self.findNeighbouringNodes(child, point)        

    #find euclidean distance between a node and an XY point (DONE)
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist
    
    #check if the goal is within stepsize (rho) distance from point, return true if so otherwise false
    def goalFound(self,point):
        #if(self.goal.locationX <= self.rho + point[0] and self.goal.locationY <= self.rho + point[1]):
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False
    
    #reset: set nearestNode to None, nearestDistance to 10K and neighbouringNodes to empty array
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
        self.neighbouringNodes = []
    
    #trace the path from goal to start, since have to reset if called many times, do this iteratively
    def retracePath(self):
        self.numWaypoints = 0
        self.Waypoints = []
        goalCost = 0
        goal = self.goal
        while goal.locationX != self.randomTree.locationX:
            #add 1 to numWaypoints
            #extract the X Y location of goal in a numpy array 
            #insert this array to waypoints (from the beginning)
            #add distance between the node and it's parent to goalCost (goalCost keeps increasing)
            self.numWaypoints += 1
            currentPoint = np.array([goal.locationX, goal.locationY])
            self.Waypoints.insert(0, currentPoint)
            # goalCost += self.distance(p1, p1.parent)
            goalCost += self.distance(goal, np.array([goal.parent.locationX, goal.parent.locationY]))
            goal = goal.parent  #set the node to it's parent
        self.goalCosts.append(goalCost)    
        
    #find unique path length from root of a node (cost) (DONE)
    def findPathDistance(self, node):
        costFromRoot = 0
        currentNode = node
        while currentNode.locationX != self.randomTree.locationX:
            costFromRoot += self.distance(currentNode, np.array([currentNode.parent.locationX, currentNode.parent.locationY])) 
            currentNode = currentNode.parent   
        return costFromRoot    
        
#end of class definitions
#----------------------------------------------------------------------------------------------------------------------------#
        
#load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([300.0, 300.0])
goal = np.array([1400.0, 775.0])
numIterations = 100
stepSize = 75
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("RRT Star Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
    
#Begin
rrtStar = RRTStarAlgorithm(start, goal, numIterations, grid, stepSize)
plt.pause(2)

#RRT Star algorithm
#iterate
for i in range(rrtStar.iterations):
    
    #Reset nearest values, call the resetNearestValues method
    print("Iteration: ",i)
    
    #algorithm begins here-------------------------------------------------------------------------------
    
    #sample a point (use the appropriate method)
    point = rrtStar.sampleAPoint()
    #find the nearest node w.r.t to the point (just call the method do not return anything)
    rrtStar.findNearest(rrtStar.randomTree, point)
    #steer to point, set the returned variable to ('new')
    new = rrtStar.steerToPoint(rrtStar.nearestNode, point)
    #if not in obstacle    
    if not rrtStar.isInObstacle(rrtStar.nearestNode, new):
        rrtStar.findNeighbouringNodes(rrtStar.randomTree, new)
        min_cost_node = rrtStar.nearestNode
        min_cost = rrtStar.findPathDistance(min_cost_node)
        min_cost = min_cost + rrtStar.distance(rrtStar.nearestNode, new)
        
        #connect along minimum cost path
        #for each node in neighbouringNodes
            #find the cost from the root (findPathDistance)
            #add the distance between the node and the new point ('new') to the above cost (use the relevant method)
        for vertex in rrtStar.neighbouringNodes:
            vertex_cost = rrtStar.findPathDistance(vertex)
            vertex_cost = vertex_cost + rrtStar.distance(vertex, new)
        #if node and new are obstacle free AND the cost is lower than min_cost (use the relevant method)
            #set the min_cost_node to this node
            #set the min_cost to this cost
            if not rrtStar.isInObstacle(vertex, new) and vertex_cost < min_cost:
                min_cost_node = vertex
                min_cost = vertex_cost
        #update nearest node to min_cost_node, create a treeNode object from the new point - call this newNode ('new[0],new[1]')
        #SIDE. If neighbouringNodes is empty, it'll add to the original nearest node (obstacle free)  
        #addChild (add newNode to the nearest node - which is now updated and is the minimum cost node)
        rrtStar.nearestNode = min_cost_node
        newNode = treeNode(new[0], new[1])
        rrtStar.addChild(newNode)
        
        #plot for display
        plt.pause(0.01)
        plt.plot([rrtStar.nearestNode.locationX, new[0]], [rrtStar.nearestNode.locationY, new[1]],'go', linestyle="--")  
        
        #rewire tree   
        #for each node in neighbouringNodes
            #set a variable: 'cost' to min_cost
            #add the distance between 'new' and node to cost
            #if node and new are obstacle free AND the cost is lower than the distance from root to vertex (use findPathDistance method)
            #set the parent of node to 'newNode' (see line 190)
        for vertex in rrtStar.neighbouringNodes:
            vertex_cost = min_cost
            vertex_cost = vertex_cost + rrtStar.distance(vertex, new)
            if not rrtStar.isInObstacle(vertex, new) and vertex_cost < rrtStar.findPathDistance(vertex):
                vertex.parent = newNode
            
        #if goal found, and the projected cost is lower, then append to path let it sample more (DONE)
        point = np.array([newNode.locationX, newNode.locationY])
        if rrtStar.goalFound(point):
            projectedCost = rrtStar.findPathDistance(newNode) + rrtStar.distance(rrtStar.goal, point)
            if projectedCost < rrtStar.goalCosts[-1]:
                rrtStar.addChild(rrtStar.goal)
                plt.plot([rrtStar.nearestNode.locationX, rrtStar.goalArray[0]], [rrtStar.nearestNode.locationY, rrtStar.goalArray[1]],'go', linestyle="--") 
                #retrace and plot, this method finds waypoints and cost from root
                rrtStar.retracePath()
                print("Goal Cost: ", rrtStar.goalCosts)
                plt.pause(0.25)
                rrtStar.Waypoints.insert(0,start)
                #plot the waypoints
                for i in range(len(rrtStar.Waypoints)-1):
                    plt.plot([rrtStar.Waypoints[i][0], rrtStar.Waypoints[i+1][0]], [rrtStar.Waypoints[i][1], rrtStar.Waypoints[i+1][1]],'ro', linestyle="--")
                    plt.pause(0.01)

print("Goal Costs: ", rrtStar.goalCosts) 
