import pygame
import time
import numpy as np
import random

# Colors
white = (255,255,255)
black = (0,0,0)
green = (0,255,0)
red = (255,0,0)

# Control constraints
delta_t = 10.0
delta_t_sec = delta_t/1000.0
delta_t_control = 0.5

v_max = 100.0       # pixels per second
omega_max = 360.0   # degrees per second

n = 50000
epsilon = 40.0
sample = 10.0

obs = 2

## Initialize pygame
pygame.init()
pygame.display.set_caption('Kinodynamic RRT')

Length = 1200
Width = 800
screen = pygame.display.set_mode((Length,Width))

ddr = pygame.image.load('ship1.png')
ddr_pos = np.array([100.0, 100.0])
ddr_angle = 0.0
ddr_diam = 80.0
ddr_radius = int(ddr_diam/2)

def draw_DDR(pos, angle):
    angle_ = angle - 90.0
    pos_new = np.array([int(round(pos[0]-32)), int(round(pos[1]-32))])

    orig_rect = ddr.get_rect()
    rot_image = pygame.transform.rotate(ddr, angle_)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()

    screen.blit(rot_image, pos_new)

def draw_Obstacles(scene):
    if scene == 1:
        pygame.draw.line(screen, black, (200, 200), (500, 200), 100)
        pygame.draw.line(screen, black, (700, 400), (1000, 400), 300)
        pygame.draw.line(screen, black, (300, 300), (300, 600), 50)
    elif scene == 2:
        pygame.draw.line(screen, black, (150, 0), (150, 300), 50)
        pygame.draw.line(screen, black, (150, 600), (150, 800), 50)
        pygame.draw.line(screen, black, (400, 0), (400, 600), 50)
        pygame.draw.line(screen, black, (700, 300), (700, 800), 50)
        pygame.draw.line(screen, black, (900, 0), (900, 500), 50)
    elif scene == 0:
        pass

def free_sourranding(p):
    samples = 24
    angle = 0.0
    p_ = np.array([int(round(p[0])), int(round(p[1]))])
    for i in range(0,samples):
        p1 = p_ + np.asarray((int(ddr_radius*np.cos(np.deg2rad(angle))), int(ddr_radius*np.sin(np.deg2rad(angle)))))

        if 0<=p1[0]<Length and 0<=p1[1]<Width:
            if screen.get_at(p_) == white and screen.get_at(p1) == white:
                valid = True
            else:
                valid = False
                break
        else:
            valid = False
            break
        if valid == False:
            break

        angle += 360/samples
    
    return valid

# Set of controls
def U_set(sel):
    u = np.zeros(2)
    if sel == 0:
        u[0] = 0.0
        u[1] = 0.0
    elif sel == 1:
        u[0] = 1.0
        u[1] = 1.0
    elif sel == 2:
        u[0] = 1.0
        u[1] = 0.0
    elif sel == 3:
        u[0] = 0.0
        u[1] = 1.0
    elif sel == 4:
        u[0] = -1.0
        u[1] = -1.0
    elif sel == 5:
        u[0] = -1.0
        u[1] = 0.0
    elif sel == 6:
        u[0] = 0.0
        u[1] = -1.0
    elif sel == 7:
        u[0] = -1.0
        u[1] = 1.0
    elif sel == 8:
        u[0] = 1.0
        u[1] = -1.0
    
    return u

# Collition check function
def coll_check(p_init, angle_init, u_sel):
    p_new = np.array([p_init[0], p_init[1]])
    angle_new = angle_init

    new_delta = delta_t_control/sample
    u_ = U_set(u_sel)

    all_points = []
    all_angles = []

    for i in range(0, int(sample)):
        p_new[0] = p_new[0] + (u_[0]+u_[1])/2.0 * v_max * np.cos(np.deg2rad(angle_new)) * new_delta
        p_new[1] = p_new[1] - (u_[0]+u_[1])/2.0 * v_max * np.sin(np.deg2rad(angle_new)) * new_delta
        angle_new = angle_new + (u_[1]-u_[0])/2.0 * omega_max * new_delta

        p_new_ = np.array([int(round(p_new[0])), int(round(p_new[1]))])

        all_points.append(p_new)
        all_angles.append(angle_new)

        if 0<=p_new_[0]<Length and 0<=p_new_[1]<Width:
            if screen.get_at(p_new_) == black or free_sourranding(p_new) == False: 
                return True, 0, 0
    
    return False, all_points, all_angles

# Euclidean distance function
def dist(x_,y_):
    return np.sqrt((y_[0]-x_[0])**2 + (y_[1]-x_[1])**2)

## RRT Algorithm
def RRT(x_init, angle_init, x_goal):
    goal_reached = False
    T = []

    x_new = []
    x_new.append(x_init)
    x_new.append(angle_init)
    x_new.append(-1)
    x_new.append(0)

    T.append(x_new)

    for i in range(0, n):
        x = random.randint(0,Length - 1)
        y = random.randint(0,Width - 1)
        x_rand = np.array([x, y])

        # Find closest node in the tree
        d_node = float('inf')
        for k in range(len(T)):
            if k == 0:
                if dist(x_rand, T[k][0]) < d_node:
                    x_near = k
                    d_node = dist(x_rand, T[k][0])
            else:
                if dist(x_rand, T[k][0][-1]) < d_node:
                    x_near = k
                    d_node = dist(x_rand, T[k][0][-1])

        # Random control
        rand_u = random.randint(1, 8)

        # Check if this control is valid
        if x_near == 0:
            test = coll_check(T[x_near][0], T[x_near][1], rand_u)
        else:
            test = coll_check(T[x_near][0][-1], T[x_near][1][-1], rand_u)
            
        if test[0] == False:
            x_new = []
            x_new.append(test[1])
            x_new.append(test[2])
            x_new.append(rand_u)
            x_new.append(x_near)

            T.append(x_new)

            if dist(test[1][-1], x_goal) < epsilon:
                goal_reached = True
                break

    traj = []
    angles = []
    if goal_reached:
        traj.append(T[-1][0])
        angles.append(T[-1][1])
        parent = T[-1][3]
        
        while parent != 0:
            traj.append(T[parent][0])
            angles.append(T[parent][1])
            parent = T[parent][3]
    
    traj.reverse()
    angles.reverse()

    print('Total nodes used', i)
    return goal_reached, traj, angles



# Game loop
running = True
state = 0
valid = valid2 = False
goal = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_pressed()[0] and state == 0 and valid == True:
            ddr_pos = np.asarray(pygame.mouse.get_pos())
            ddr_pos = ddr_pos.astype('float64')
            print('x_init', ddr_pos)
            state = 1
        if pygame.mouse.get_pressed()[0] and state == 1 and valid2 == True:
            pygame.draw.circle(screen, red, goal, ddr_radius)
            print('x_goal', goal)
            state = 2
        if pygame.key.get_pressed()[273] == 1 and state == 0:
            ddr_angle = 90.0
        if pygame.key.get_pressed()[274] == 1 and state == 0:
            ddr_angle = 270.0
        if pygame.key.get_pressed()[275] == 1 and state == 0:
            ddr_angle = 0.0
        if pygame.key.get_pressed()[276] == 1 and state == 0:
            ddr_angle = 180.0


    # Environment
    screen.fill(white)
    draw_Obstacles(obs)
    
    if state == 0:  # Place ship
        p = np.asarray(pygame.mouse.get_pos())
        valid = free_sourranding(p)
            
        if valid:
            draw_DDR(p, ddr_angle)

    elif state == 1:    # Place goal
        goal = np.asarray(pygame.mouse.get_pos())
        valid2 = free_sourranding(goal)
            
        if valid2:
            pygame.draw.circle(screen, red, goal, ddr_radius)
        
        draw_DDR(ddr_pos, ddr_angle)

    elif state == 2:
        start_time = time.time()
        reached, Traj, Ang = RRT(ddr_pos, ddr_angle, goal)
        elapsed_time = time.time() - start_time

        print('Time to compute trajectorie', elapsed_time)

        if reached:
            start_time = time.time()
            for k in range(0,len(Traj)):
                for j in range(0,int(sample)):
                    ddr_pos = Traj[k][j]
                    ddr_angle = Ang[k][j]

                    screen.fill(white)
                    draw_Obstacles(obs)

                    pygame.draw.circle(screen, red, goal, ddr_radius)
                    draw_DDR(ddr_pos, ddr_angle)

                    pygame.display.update()
                    pygame.time.delay(int(delta_t_control*1000/sample))
                
            elapsed_time = time.time() - start_time
            print('Time to get to goal', elapsed_time)
            
            print('Goal reached')

        else:
            print('Goal not reached')

        state = 3
    elif state == 3:
        pygame.draw.circle(screen, red, goal, ddr_radius)
        draw_DDR(ddr_pos, ddr_angle)

    
    pygame.display.update()
    pygame.time.delay(int(delta_t))
