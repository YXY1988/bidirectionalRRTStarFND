import random
import math
from sqlite3 import Timestamp
import sys
import pygame
import timeit, time
import numpy as np
show_animation = True

XDIM = 1740 #34780/20
YDIM = 832  #15400/20
windowSize = [XDIM, YDIM]
cabin_width=140  #2800/20 
cabin_height=325 #6500/20
polar_width=8    #160/20
light_width=10    #200/20 #吊灯 

pygame.init()
fpsClock = pygame.time.Clock()

screen = pygame.display.set_mode(windowSize)
pygame.display.set_caption('PMCU 推舱路径规划')


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=15.0, goalSampleRate=10, maxIter=1500):

        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.Xrand = randArea[0]
        self.Yrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """
        # 初始化节点列表，将作为第一个节点
        self.nodeList = {0:self.start}
        i = 0

        # 循环生成新节点，直到找到路径或达到最大迭代次数
        while True:
            # print(i)
            time_st=time.time()
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(rnd) # get nearest node index to random point
            newNode = self.steer(rnd, nind) # generate new node from that nearest node in direction of random point

            # collision detection
            if self.__CollisionCheck(newNode, self.obstacleList): # if it does not collide

                nearinds = self.find_near_nodes(newNode, 5) # find nearest nodes to newNode
                newNode = self.choose_parent(newNode, nearinds) # from that nearest nodes find the best parent to newNode
                self.nodeList[i+100] = newNode # add newNode to nodeList
                self.rewire(i+100, newNode, nearinds) # make newNode a parent of another node if necessary
                self.nodeList[newNode.parent].children.add(i+100)

                if len(self.nodeList) > self.maxIter:
                    leaves = [ key for key, node in self.nodeList.items() if len(node.children) == 0 and len(self.nodeList[node.parent].children) > 1 ]
                    if len(leaves) > 1:
                        ind = leaves[random.randint(0, len(leaves)-1)]
                        self.nodeList[self.nodeList[ind].parent].children.discard(ind)
                        self.nodeList.pop(ind)
                    else:
                        leaves = [ key for key, node in self.nodeList.items() if len(node.children) == 0 ]
                        ind = leaves[random.randint(0, len(leaves)-1)]
                        self.nodeList[self.nodeList[ind].parent].children.discard(ind)
                        self.nodeList.pop(ind)
            
            time_ed=time.time()
            curr_time=time_ed-time_st
            i+=1
            
            # draw fig every 25
            if animation and i%25 == 0:
                self.DrawGraph(rnd)

            for e in pygame.event.get():
                if e.type == pygame.MOUSEBUTTONDOWN:
                    if e.button == 1: # left btn, add a new obstacle
                        pass
                        self.obstacleList.append((e.pos[0],e.pos[1],cabin_width,cabin_height))
                        self.path_validation()
                        
                    elif e.button == 3: # right btn, update the final target positon
                        self.end.x = e.pos[0]
                        self.end.y = e.pos[1]
                        self.path_validation()

    def path_validation(self):
        lastIndex = self.get_best_last_index()
        if lastIndex is not None:
            while self.nodeList[lastIndex].parent is not None:
                nodeInd = lastIndex
                lastIndex = self.nodeList[lastIndex].parent

                dx = self.nodeList[nodeInd].x - self.nodeList[lastIndex].x
                dy = self.nodeList[nodeInd].y - self.nodeList[lastIndex].y
                d = math.sqrt(dx ** 2 + dy ** 2)
                theta = math.atan2(dy, dx)
                if not self.check_collision_extend(self.nodeList[lastIndex].x, self.nodeList[lastIndex].y, theta, d):
                    self.nodeList[lastIndex].children.discard(nodeInd)
                    self.remove_branch(nodeInd)

    def remove_branch(self, nodeInd):
        for ix in self.nodeList[nodeInd].children:
            self.remove_branch(ix)
        self.nodeList.pop(nodeInd)

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i].x, self.nodeList[i].y, theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))


        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind
        return newNode

    def steer(self, rnd, nind):

        # expand tree
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = Node(nearestNode.x, nearestNode.y)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost = nearestNode.cost + self.expandDis
        newNode.parent = nind 
        return newNode

    def get_random_point(self):
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(0, self.Xrand), random.uniform(0, self.Yrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]
        return rnd

    def get_best_last_index(self):

        disglist = [(key, self.calc_dist_to_goal(node.x, node.y)) for key, node in self.nodeList.items()]
        goalinds = [key for key, distance in disglist if distance <= self.expandDis]

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[key].cost for key in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode, value):
        r = self.expandDis * value

        dlist = np.subtract( np.array([ (node.x, node.y) for node in self.nodeList.values() ]), (newNode.x,newNode.y))**2
        dlist = np.sum(dlist, axis=1)
        nearinds = np.where(dlist <= r ** 2)
        nearinds = np.array(list(self.nodeList.keys()))[nearinds]

        return nearinds

    def rewire(self, newNodeInd, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode.x, nearNode.y, theta, d):
                    self.nodeList[nearNode.parent].children.discard(i)
                    nearNode.parent = newNodeInd
                    nearNode.cost = scost
                    newNode.children.add(i)

    def check_collision_extend(self, nix, niy, theta, d):

        tmpNode = Node(nix,niy)

        for i in range(int(d/5)):
            tmpNode.x += 5 * math.cos(theta)
            tmpNode.y += 5 * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        screen.fill((255, 255, 255))
        for node in self.nodeList.values():
            if node.parent is not None:
                pygame.draw.line(screen,(0,255,0),[self.nodeList[node.parent].x,self.nodeList[node.parent].y],[node.x,node.y], width=2)

        for node in self.nodeList.values():
            if len(node.children) == 0: 
                pygame.draw.circle(screen, (255,0,255), [int(node.x),int(node.y)], 2)
                
        for(sx,sy,ex,ey) in self.obstacleList:
            pygame.draw.rect(screen,(220,0,0), [(sx,sy),(ex,ey)])

        # yxy: use rect shape instead of the circle shape of begin and end pose
        # pygame.draw.circle(screen, (255,0,0), [self.start.x, self.start.y], 10) 
        # pygame.draw.circle(screen, (0,0,255), [self.end.x, self.end.y], 10)
        pygame.draw.rect(screen,(0,0,255),[(self.start.x,self.start.y),(cabin_width,cabin_height)])
        pygame.draw.rect(screen,(0,255,100),[(self.end.x,self.end.y),(cabin_width,cabin_height)])
        lastIndex = self.get_best_last_index()
        if lastIndex is not None:
            path = self.gen_final_course(lastIndex)
            ind = len(path)
            final_dist=0;
            for i in range(0,ind-1):
                delta_x=path[i+1][0]-path[i][0]
                delta_y=path[i+1][1]-path[i][1]
                final_dist=final_dist+math.sqrt(pow(delta_x,2)+pow(delta_y,2))
            #print('the final dist is: ',final_dist)
            while ind > 1:
                pygame.draw.line(screen,(255,0,0),path[ind-2],path[ind-1],width=5)
                ind-=1

        pygame.display.update()

    def GetNearestListIndex(self, rnd):
        dlist = np.subtract( np.array([ (node.x, node.y) for node in self.nodeList.values() ]), (rnd[0],rnd[1]))**2
        dlist = np.sum(dlist, axis=1)
        minind = list(self.nodeList.keys())[np.argmin(dlist)]
        return minind

    #yxy: 此处node是点，obstacle是rect, 如扩展到推舱，应将node也改为 rect
    def __CollisionCheck(self, node, obstacleList):

        for(sx,sy,ex,ey) in obstacleList:
            sx,sy,ex,ey = sx+2,sy+2,ex+2,ey+2
            if node.x > sx and node.x < sx+ex:
                if node.y > sy and node.y < sy+ey:
                    return False

        return True  # safe

class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.children = set()

def main():
    print("start RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        # polars x 3
        (400, 800, polar_width, polar_width),
        (800, 800, polar_width, polar_width),
        (1200, 800, polar_width, polar_width),
        # lights x 6
        (50, 432, light_width, light_width),
        (450, 432, light_width, light_width),
        (750, 432, light_width, light_width),
        (1050, 432, light_width, light_width),
        (1350, 432, light_width, light_width),
        (1650, 432, light_width, light_width),
        # unavailable areas x 2
        (620,475,0.5*cabin_width,cabin_height),
        (1110,475,350,350)
    ]  # [x,y,w,h]
    InstalledList = [] #yxy: installed should also be viewed as obstacle, prepared for outfittings
    # Set Initial parameters
    rrt = RRT(start=[105, 0], goal=[540, 150],
              randArea=[XDIM, YDIM], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

if __name__ == '__main__':
    main()
 