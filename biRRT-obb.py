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
                 randArea, expandDis=5, goalSampleRate=50, maxIter=2000):#15,10,1500

        self.start = Node(start[0], start[1],cabin_width,cabin_height,0)
        self.end = Node(goal[0], goal[1],cabin_width,cabin_height,0)
        self.Xrand = randArea[0]
        self.Yrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.new_obstacles = []
        self.clear_new_obstacles=False

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
            rnd = self.get_random_point() #step 1 
            nind = self.GetNearestListIndex(rnd) # get nearest node index to random point
            newNode = self.steer(rnd, nind) # generate new node from that nearest node in direction of random point

            # collision detection
            if self.__CollisionCheckObb(newNode, self.obstacleList): # if it does not collide

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
                        obstacle = (e.pos[0], e.pos[1], cabin_width, cabin_height)
                        self.obstacleList.append(obstacle)
                        self.new_obstacles.append(obstacle)  # 将新增的障碍物添加到新列表中
                        self.path_validation()
                        
                    elif e.button == 3: # right btn, update the final target positon
                        self.end.x = e.pos[0]
                        self.end.y = e.pos[1]
                        self.path_validation()
                elif e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_r:  # clear only the newly added obstacles
                        for obstacle in self.new_obstacles:
                            if obstacle in self.obstacleList:
                                self.obstacleList.remove(obstacle)  # 仅移除新增加的障碍物
                        self.new_obstacles.clear()  # 清空新障碍物列表
                        self.clear_new_obstacles = True # 通知Draw刷新 
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
                    self.remove_branch(nodeInd) #尝试不移出分支

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
        newNode = Node(nearestNode.x, nearestNode.y,cabin_width,cabin_height,0)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost = nearestNode.cost + self.expandDis
        newNode.parent = nind 
        return newNode

    def get_random_point(self):
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(0, self.Xrand), random.uniform(0, self.Yrand)]
        else:  # goal point sampling
            # rnd = [self.end.x-random.uniform(0, 30), self.end.y-random.uniform(0,100)]
            rnd=[self.end.x,self.end.y]
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

        tmpNode = Node(nix,niy,cabin_width,cabin_height,theta)
        if not self.__CollisionCheckObb(tmpNode, self.obstacleList):
            return False
        
        # for i in range(int(d/5)):
        #     tmpNode.x += 5 * math.cos(theta)
        #     print(math.cos(theta))
        #     tmpNode.y += 5 * math.sin(theta)
        #     if not self.__CollisionCheckObb(tmpNode, self.obstacleList):
        #         return False

        return True

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        # background
        screen.fill((255, 255, 255))
        
        # draw green path 
        for node in self.nodeList.values():
            if node.parent is not None:
                pygame.draw.line(screen,(0,255,0),[self.nodeList[node.parent].x,self.nodeList[node.parent].y],[node.x,node.y])

        # draw purple nodes
        for node in self.nodeList.values():
            if len(node.children) == 0: 
                pygame.draw.circle(screen, (255,0,255), [int(node.x),int(node.y)], 2)
        
        #draw obstacles
        for(sx,sy,ex,ey) in self.obstacleList:
            if not self.clear_new_obstacles or (sx,sy,ex,ey) not in self.new_obstacles:
                pygame.draw.rect(screen,(220,0,0), [(sx,sy),(ex,ey)])

        #draw start and end
        pygame.draw.rect(screen,(0,0,255),[(self.start.x,self.start.y),(cabin_width,cabin_height)])
        pygame.draw.rect(screen,(0,255,100),[(self.end.x,self.end.y),(cabin_width,cabin_height)])
        lastIndex = self.get_best_last_index()
        if lastIndex is not None:
            path = self.gen_final_course(lastIndex)
            ind = len(path)
            final_dist=0
            for i in range(0,ind-1):
                delta_x=path[i+1][0]-path[i][0]
                delta_y=path[i+1][1]-path[i][1]
                final_dist=final_dist+math.sqrt(pow(delta_x,2)+pow(delta_y,2))
            #print('the final dist is: ',final_dist)
            while ind > 1:
                pygame.draw.line(screen,(255,0,0),path[ind-2],path[ind-1],width=4)
                ind-=1

        pygame.display.update()

    def GetNearestListIndex(self, rnd):
        dlist = np.subtract( np.array([ (node.x, node.y) for node in self.nodeList.values() ]), (rnd[0],rnd[1]))**2
        dlist = np.sum(dlist, axis=1)
        minind = list(self.nodeList.keys())[np.argmin(dlist)]
        return minind

    #yxy: 此处node是点，obstacle是rect, 如扩展到推舱，应将node也改为 rect
    def __CollisionCheck(self, node, obstacleList):

        for(sx,sy,width,height) in obstacleList:
            sx,sy,width,height = sx+2,sy+2,width+2,height+2
            if node.x > sx and node.x < sx+width:
                if node.y > sy and node.y < sy+height:
                    return False

        return True  # safe
    
    def __CollisionCheckObb(self,node,obstacleList):
        for(sx,sy,width,height) in obstacleList:
            ObstacleNode=Node(sx,sy,width,height,0)
            if self.separating_axis_theorem(node,ObstacleNode):
                return False
            if self.is_out_bounds(node.get_corners(),XDIM,YDIM):
                return False 
        return True # safe

    def separating_axis_theorem(self,obb1, obb2):
    # 使用分离轴定理检测 OBB 碰撞
        def get_projection(corners, axis):
            min_proj = float('inf')
            max_proj = float('-inf')
            for corner in corners:
                proj = corner[0] * axis[0] + corner[1] * axis[1]
                min_proj = min(min_proj, proj)
                max_proj = max(max_proj, proj)
            return min_proj, max_proj

        def overlap(proj1, proj2):
            return proj1[0] <= proj2[1] and proj2[0] <= proj1[1]

        axes = []
        corners1 = obb1.get_corners()
        corners2 = obb2.get_corners()

        for i in range(4):
            edge = (corners1[i][0] - corners1[i - 1][0], corners1[i][1] - corners1[i - 1][1])
            axis = (-edge[1], edge[0])
            axes.append(axis)

        for i in range(4):
            edge = (corners2[i][0] - corners2[i - 1][0], corners2[i][1] - corners2[i - 1][1])
            axis = (-edge[1], edge[0])
            axes.append(axis)

        for axis in axes:
            proj1 = get_projection(corners1, axis)
            proj2 = get_projection(corners2, axis)
            if not overlap(proj1, proj2):
                return False

        return True

    def is_out_bounds(self,corners,xlim,ylim):
        for (x,y) in corners:
            if x < 0 or x > xlim or y < 0 or y > ylim:
               return True #out bound
        return False # in bound

class Node():
    """
    RRT Node
    """

    def __init__(self, x, y,  width, height, angle):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.angle = angle

        self.cost = 0.0
        self.parent = None
        self.children = set()

    def get_corners(self):
        # 计算 OBB 的四个角点
        cos_a = math.cos(math.radians(self.angle))
        sin_a = math.sin(math.radians(self.angle))
        hw = self.width / 2
        hh = self.height / 2

        corners = [
            (self.x, self.y),
            (self.x + self.width * cos_a, self.y + self.width * sin_a),
            (self.x + self.width * cos_a - self.height * sin_a, self.y + self.width * sin_a + self.height * cos_a),
            (self.x - self.height * sin_a, self.y + self.height * cos_a)
        ]
        return corners

    def draw(self, screen, color):
        corners = self.get_corners()
        pygame.draw.polygon(screen, color, corners, 2)

def main():
    print("start RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        # polars x 3
        (400, 800, polar_width, polar_width), # 因 polar和light都视为正方形，所以都用width
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
    rrt = RRT(start=[105, 0], goal=[620+0.5*cabin_width+10, 475-5],
              randArea=[XDIM, YDIM], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

if __name__ == '__main__':
    main()
 