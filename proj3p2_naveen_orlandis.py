import numpy as np
import math
import cv2
from math import dist
import time

fourcc = cv2.VideoWriter_fourcc(*'XVID')                    
out = cv2.VideoWriter('project_4.avi', fourcc, 30.0, (600, 250))

#Creating work space
b_canvas = np.zeros((250,600,3),np.uint8)
width = 600
height = 250

thresh = 0.5
angle_thresh = 30
x_axis = np.arange(0, 600, thresh)
y_axis = np.arange(0, 250, thresh)
theta_grid = np.arange(0, 360, angle_thresh)
theta_move = [-60, -30, 0, 30, 60]

####################################Creating obstacle space
def w_space(max_x,max_y, clear):
    #Storing points of the array into an empty list
    all_points = []
    obstacle_space = []
    for i in range(0,max_x):
        for j in range(0,max_y): 
            all_points.append((i,j)) #Appending all points to the list
    for e in all_points:
        x = e[1]
        y = e[0]
    #Defining obstacles using Half - Plane equations
    #Lower rectangle
        if y>=(100-clear) and y<=(150+clear) and x>=0 and x<=(100+clear):
            obstacle_space.append((x,y))
        elif y>=(100-clear) and y<=(150+clear) and x>=(150-clear) and x<=250: #Upper rectangle
            obstacle_space.append((x,y))
        if (y >= (460 - clear)) and (x >= (25 - clear)) and (x <= (225 + clear)) and ((-2*y + x) >= -895 + clear) and ((2*y + x) <= 1145 - clear):#Triangle
            obstacle_space.append((x,y))
        elif (y >= (235 - clear)) and (y <= (365 + clear)) and ((y + 2*x) >= 390 - clear) and ((y - 2*x) <= 210 + clear) and ((y - 2*x) >= -130 + clear) and ((y + 2*x) <= 710 + clear): #Hexagon
            obstacle_space.append((x,y))
    #Puffing the walls
    for i in range(clear):
        for j in range(b_canvas.shape[0]):
            for k in range(b_canvas.shape[1]):
                b_canvas[j][i] = (0,0,255)
                b_canvas[i][k] = (0,0,255)
                b_canvas[b_canvas.shape[0]-1-i][k] = (0,0,255)
                b_canvas[j][b_canvas.shape[1]-1-i] = (0,0,255)
    for c in obstacle_space: 
        x = c[0]
        y = c[1]
        b_canvas[(x,y)]=[0,0,255] #Coloring the obstacles

    ch1, ch2, ch3 = cv2.split(b_canvas)
    ch3 = ch3.T

    return (obstacle_space,ch3)

####################################Creating nodes and storing in a dictionary
def get_node( pos, theta, parent, cost):
    Node  = {'pos': pos,
             'theta': theta, 
             'parent': parent, 
             'cost': cost}
    return Node

def initial_nodes(start_key):
  open_dict = {}
  for x in x_axis: 
    for y in y_axis:  
        pos = (x, y)
        for theta in theta_grid:  
            open_dict[(pos, theta)] = get_node(pos, theta, None, np.inf) #Adding key inside key
  open_dict[start_key]['cost'] = 0 #Assigning initial cost to be zero
  return open_dict
################################Goal check
def is_goal(child_node, goal_pos, theta_goal):
    dst = dist(child_node['pos'], goal_pos) #Calculating Euclidean distance
    
    dtheta = np.abs(child_node['theta'] - theta_goal)
    if dst <= 1.5 and dtheta <= 30: #Goal reached
        return  True
    else: 
        return False
    
##############################Back tracking
def backtrack(b_node):
    print("Generating optimal path")
    path = []
    while b_node['parent'] is not None:
        path.append(b_node)
        b_node = b_node['parent']
    path.reverse()
    return path

#############################Defining movemnet
def action(node_, theta, rpm1, rpm2):
    t = 0
    dt = 0.1
    r = 0.038
    L = 0.354
    x, y = node_['pos']
    x_list = []
    y_list = []
    #n_theta = (node_['theta'] + theta)%360
    r_theta = np.deg2rad(theta)
    while t<1:
        x_ = 0.5*r * (rpm1 + rpm2) * math.cos(r_theta) * dt + x
        y_ = 0.5*r * (rpm1 + rpm2) * math.sin(r_theta) * dt + y
        n_theta = (r / L) * (rpm2 - rpm1) * dt + n_theta
        cost = dist([x,y],[x_,y_]) + cost
        x_list.append(x_)
        y_list.append(y_)
    n_theta = np.rad2deg(n_theta)
    f_theta = n_theta%360  

    for i in x_list:
        for j in y_list:
            if b_canvas[i][j] == (0,0,255):
                break


    #n_pos = ((x_//thresh)//2, (y_//thresh)//2)
    return n_pos, n_theta, node_['cost'] + 1

 ###########################Checking if child node in obstacle path
def o_space(pose):
    x = int(pose[0])
    y = int(pose[1])
    if ch3[x][y] == 255:
        return True
    else:
        return False
#######################################
print("---------------------------------------------------------")
print("Path Planner | Workspace dimensions : 250X600 pixels")
print("Accepted orinetation of the robot : -360 to 360: as mutiples of 30")
print("Accepted step size : 1 to 10")
    
ip = True    
while ip:

    print("---------------------------------------------------------")
    start_x= int(input("Enter the x coordinate of the start point: "))
    start_y= int(input("Enter the y coordinate of the start ponit: "))
    theta_s = int(input("Enter the orientation of the start point: "))
    goal_x= int(input("Enter the x coordinate of the goal point: "))
    goal_y= int(input("Enter the y coordinate of the goal point: "))
    l_rpm = int(input("Enter Left wheel's RPM: "))
    r_rpm = int(input("Enter Right wheels's RPM: "))
    cl = int(input("Enter the clearance of the robot "))

    clearance = cl

    obstacles, ch3 = w_space(width,height,clearance)

    if (theta_s % 30 != 0):
        print('Invalid entry of orientation')
        print("Try again")
    elif (theta_g % 30 != 0):
        print("Invalid entry of orientation")
        print("Try again")
    elif k not in range(0,11):
        print("Enter K in the range of 1 to 10; Try again")
    elif (start_x > b_canvas.shape[1] or start_y > b_canvas.shape[0] or goal_x > b_canvas.shape[1] or goal_y > b_canvas.shape[0]): #Checking whether outside the work space
        print("Invalid input, entered value outside the path space")
        print("Try Agian")
    elif ch3[start_x][start_y] == 255 or ch3[goal_x][goal_y] == 255: #Checking for obstacles
        print("Invalid input, entered value in obstacle space")
        print("Try Again")
    else:
        ip = False

if theta_s >= 360:
        theta_s = theta_s - 360
if theta_s >= -360 and theta_s < 0:
        theta_s = theta_s + 360
    
s_node = (start_x,start_y)
g_node = (goal_x, goal_y)
s_node_key = (s_node,theta_s)
nodes = initial_nodes(s_node_key)
start_time = time.time()
actions = [[0,l_rpm],[l_rpm,0],[l_rpm,l_rpm],[0,r_rpm],[r_rpm,0],[r_rpm,r_rpm],[l_rpm,r_rpm],[r_rpm,l_rpm]] 

def a_star(g_node, theta_g):
    open_dict = {s_node_key: dist(s_node, g_node)}
    c_list = {s_node_key}
    create = [nodes[s_node_key]] #Contains initial information

    while len(open_dict): #Loop until open list is empty
        key = min(open_dict, key = open_dict.get) #Finds the node with the smallest distance
        c_list.add(key)
        open_dict.pop(key)
        m_node = nodes[key]
        if is_goal(m_node, g_node, theta_g):
            print("Goal reached")
            return backtrack(child), create
        for act in actions:
            pos, theta, cost = action(m_node, theta_s, act[0], act[1]) #New node and cost
            print("Node",pos)
            if not o_space(pos) and (pos,theta) not in c_list:
                child = nodes[(pos,theta)]
                if cost < child['cost']:
                    child['cost'] = cost
                    child['parent'] = m_node
                    open_dict[(pos, theta)] = cost + dist(pos, g_node)
                    create.append(child)

btrack , create = a_star(g_node,theta_g)
print('Execuion time ' + str(time.time() - start_time) + ' seconds') 
print("Generating video....")

cv2.circle(b_canvas, (goal_x,b_canvas.shape[0]-goal_y), 2, (255, 255, 255), 2)
cv2.circle(b_canvas, (start_x,b_canvas.shape[0]-start_y), 2, (255, 255, 255), 2)

for n in create:
    i, j = n['pos'] #Extracts the position of the current node
    parent = n['parent']
    if parent is None:
        parent = btrack[0]
    i_, j_ = parent['pos']
    cv2.arrowedLine(b_canvas, (int(i), int(249-j)), (int(i_), int(249-j_)), [0,250,0], 2)
    out.write(b_canvas)

print("Backtracked path")

if btrack is not None:
    for n in btrack:
        print(n['pos'], n['theta'])
        i, j = n['pos']
        parent = n['parent']
        if parent is None:
            parent = btrack[0]
        i_, j_ = parent['pos']
        cv2.arrowedLine(b_canvas, (int(i), int(249-j)), (int(i_), int(249-j_)), [255,0,0], 1)
        out.write(b_canvas)

out.release()
print("Video saved")
            

            





    
    