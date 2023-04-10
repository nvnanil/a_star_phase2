import numpy as np
import matplotlib.pyplot as plt
import time
import csv

def cost(Xn,Yn,theta,UL,UR):   
    t = 0
    dt = 0.1
    r = 3.3
    L = 16
    theta = 3.14 * theta / 180       
    D=0
    while t<1.4:
        t = t + dt
        Xs = Xn
        Ys = Yn
        if (obstacle(Xs,Ys)):           
            return 0,0,0,0
        Xn += 0.5*r * (UL + UR) * np.cos(theta) * dt
        Yn += 0.5*r * (UL + UR) * np.sin(theta) * dt
        theta += (r/L) * (UR - UL) * dt
        D += np.sqrt(np.power(0.5*r*(UL+UR)*np.cos(theta)*dt, 2) + np.power(0.5*r*(UL+UR)*np.sin(theta)*dt, 2))
        if(nodes):
            plt.plot([Xs, Xn], [Ys, Yn], color="blue") #Visualize the generated nodes
        if(backtrack_nodes):
            plt.plot([Xs, Xn], [Ys, Yn], color="yellow")#Visualize the optimal path
    theta = 180 * (theta) / 3.14
    return Xn, Yn, theta, D

def obstacle(x,y): #Checking for obstacles
    if(x>=150-cl)  and (x<=165+cl) and (y<=200) and (y>=75 - cl) or ((x>=250-cl) and (x<=265 + cl) and (y>=0) and (y<=125 + cl)) or (( 50 + cl >= np.sqrt((x-400)**2 + (y-110)**2))) or (x<=cl) or (x>=600-cl) or (y<=cl) or (y>=200-cl): 
        return True
def actions(open_nodee, closed_node, goal_x, goal_y):#Generating child nodes
    children = []     #(tc,c2c,c2g,parent_id,x,y,theta,rpm1,rpm2)
    actions = [[0,l_rpm],[l_rpm,0],[l_rpm,l_rpm],[0,r_rpm],[r_rpm,0],[r_rpm,r_rpm],[l_rpm,r_rpm],[r_rpm,l_rpm]] 
    for action in actions:
        x_,y_,theta,d= cost(open_nodee[5],open_nodee[6],open_nodee[7], action[0],action[1])
        if(not (x_==0 and y_==0 and theta==0 and d==0)):  # checks if there is no obstacle
            val = np.where((np.round(closed_node[:, 5] / 2) * 2 == np.round(x_ / 2) * 2) & (np.round(closed_node[:, 6] / 2) * 2 == np.round(y_ / 2) * 2)& (np.round(closed_node[:, 7] / 180) * 180 == np.round(theta / 180) * 180))[0]
            if(val.size==0):  
                c2g = np.sqrt((goal_x - x_)**2 + (goal_y - y_)**2)
                c2c = open_nodee[1]+d
                total_cost = c2g + c2c
                children.append([total_cost,c2c,c2g,open_nodee[3],x_, y_, theta,action[0],action[1]]) #Adding child to the explored list
    return children #Returns all the rpm

thresh_g = 10 #Threshold for goal checking
bot_radius = 10.5 
print("---------------------------------------------------------")
print("Path Planner | Workspace dimensions : 200X600 pixels")
print("Recommended wheel rpm of the robot : 3 to 6")
    
ip = True    
while ip:

    print("---------------------------------------------------------")
    start_x= int(input("Enter the x coordinate of the start point: "))
    start_y= int(input("Enter the y coordinate of the start ponit: "))
    theta_s = int(input("Enter the orientation of the start point: "))
    goal_x= int(input("Enter the x coordinate of the goal point: "))
    goal_y= int(input("Enter the y coordinate of the goal point: "))
    l_rpm = int(input("Enter Left wheel's RPM (recommended value 6): "))
    r_rpm = int(input("Enter Right wheels's RPM (recommended value 3): "))
    clear = int(input("Enter the clearance of the robot "))

    cl = 10.5 + clear

    if(obstacle(start_x, start_y) or obstacle(goal_x,goal_y)): #Checks whether the entered points are in the work space
        print('Invalid input: Entered coordinates in obstacle space')
        print('Try Again')
    else:
        ip = False

start_time = time.time()
nodes=0 
backtrack_nodes=0   
#Initializing the open list and closed list 
open_list = np.array([[0,0,0,1,0,start_x,start_y,theta_s,0,0]])     
closed_list = np.array([[-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]])                      
node_id = 1  
distance = np.sqrt((start_x - goal_x) ** 2 + (start_y - goal_y) ** 2)     
print('Generating the shortest path')

while( (not (distance<=thresh_g)) and (not open_list.shape[0]==0)):
    distance = np.sqrt((closed_list[-1][5] - goal_x) ** 2 + (closed_list[-1][6] - goal_y) ** 2)  
    open_list = open_list[np.argsort(open_list[:, 0])] #Sorting by total cost
    child = actions(open_list[0],closed_list, goal_x, goal_y)      
    for i in range(len(child)): 
        val = np.where((np.round(open_list[:, 5] / 2) * 2 == np.round(child[i][4] / 2) * 2) & (np.round(open_list[:, 6] / 2) * 2 == np.round(child[i][5] / 2) * 2)& (np.round(open_list[:, 7] / 180) * 180 == np.round(child[i][6] / 180) * 180))[0] #Checks whetehr child present in open list
        if(val.size>0):
            if (child[i][1] < open_list[int(val)][1]):  #Checking for the lowest cost     
                    open_list[int(val)][0] = child[i][0] #Updating the variables 
                    open_list[int(val)][1] = child[i][1]   
                    open_list[int(val)][4] = child[i][3]   
                    open_list[int(val)][8] = child[i][7]     
                    open_list[int(val)][9] = child[i][8]    
        else:
                open_list = np.vstack([open_list, [child[i][0],child[i][1],child[i][2], node_id+1,child[i][3],child[i][4],child[i][5],child[i][6],child[i][7],child[i][8]]])#Adding to open list
                node_id +=1
    closed_list = np.vstack([closed_list, open_list[0]]) #Popping  node and adding to closed list
    open_list = np.delete(open_list, 0, axis=0)

print('Reached goal')
print('Execuion time ' + str(time.time() - start_time) + ' seconds') 
nodes=1
print('Generating the graph...')
fig, ax = plt.subplots() #Plotting the graph
plt.xlim(0,600)
plt.ylim(0,200)
plt.plot(start_x, start_y, color='black', marker='o') #Marking the initial and final points
plt.plot(goal_x, goal_y, color='black', marker='o')

x, y = np.meshgrid(np.arange(0, 600), np.arange(0, 200)) #Creating the workspace
rect1 = (x>=150-(cl)) & (x<=165 + (cl)) & (y<=200) & (y>=75 - (cl)) #Upper rectangle
ax.fill(x[rect1], y[rect1], color='red')
rect2 = (x>=250-(cl)) & (x<=265 + (cl)) & (y>=0) & (y<=125 + (cl)) #Lower rectangle
ax.fill(x[rect2], y[rect2], color='red')
circle = ( 50 + (cl) >= np.sqrt((x-400)**2 + (y-110)**2)) #Circle
ax.fill(x[circle], y[circle], color='red')
boundary1 = (x<=(cl)) 
ax.fill(x[boundary1], y[boundary1], color='red')
boundary2 = (x>=600-(cl))
ax.fill(x[boundary2], y[boundary2], color='red')
boundary3 = (y<=(cl))
ax.fill(x[boundary3], y[boundary3], color='red')
boundary4 = (y>=200-(cl))
ax.fill(x[boundary4], y[boundary4], color='red')
                          
f1 = int(closed_list.shape[0]/50)
for i in range(closed_list.shape[0]):
    val = np.where(closed_list[i][4] == closed_list[:,3])[0]
    if(val.size>0):
        cost(closed_list[int(val)][5],closed_list[int(val)][6],closed_list[int(val)][7],closed_list[i][8],closed_list[i][9])
        if i % f1 == 0:
            plt.pause(0.01)
f2 = int(open_list.shape[0]/50)
for i in range(open_list.shape[0]):
    val = np.where(open_list[i][4] == open_list[:, 3])[0]
    if(val.size>0):
        cost(open_list[int(val)][5],open_list[int(val)][6],open_list[int(val)][7],open_list[i][8],open_list[i][9])
        if i % f2 == 0:
            plt.pause(0.01)

node_ = closed_list[-2][4]
left_rpm = closed_list [-2][8]
right_rpm = closed_list[-2][9]
backtrack_nodes=1
nodes=0
backtrack = np.empty((1, 5))
f3 = int(backtrack.shape[0]/50)
while (node_):
    val = np.where(closed_list[:, 3] == node_)[0]
    backtrack=np.vstack([backtrack, [closed_list[int(val)][5],closed_list[int(val)][6],closed_list[int(val)][7],left_rpm,right_rpm]])
    left_rpm=closed_list[int(val)][8]
    right_rpm=closed_list[int(val)][9]
    node_ = closed_list[int(val)][4]

backtrack = np.flip(backtrack,axis = 0)
with open('t_rpm.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    for i in range(backtrack.shape[0]):
        cost(backtrack[i][0],backtrack[i][1],backtrack[i][2],backtrack[i][3],backtrack[i][4]) #Generating the coordinates, orientation and rpm
        plt.pause(0.01)
        writer.writerow([backtrack[i][3], backtrack[i][4]]) #Storing the rpm values in csv file
plt.show()
