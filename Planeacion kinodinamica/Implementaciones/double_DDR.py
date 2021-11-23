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

ddr1 = pygame.image.load('ship1.png')
ddr1_pos = np.array([100.0, 100.0])
ddr1_angle = 0.0
ddr1_diam = 80.0
ddr1_radius = int(ddr1_diam/2)

ddr2 = pygame.image.load('ship3.png')
ddr2_pos = np.array([100.0, 100.0])
ddr2_angle = 0.0
ddr2_diam = 88.0
ddr2_radius = int(ddr2_diam/2)

def draw_DDR_1(pos, angle):
    angle_ = angle - 90.0
    pos_new = np.array([int(round(pos[0]-32)), int(round(pos[1]-32))])

    orig_rect = ddr1.get_rect()
    rot_image = pygame.transform.rotate(ddr1, angle_)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()

    screen.blit(rot_image, pos_new)
def draw_DDR_2(pos, angle):
    angle_ = angle - 90.0
    pos_new = np.array([int(round(pos[0]-32)), int(round(pos[1]-32))])

    orig_rect = ddr2.get_rect()
    rot_image = pygame.transform.rotate(ddr2, angle_)
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
        p1 = p_ + np.asarray((int(ddr1_radius*np.cos(np.deg2rad(angle))), int(ddr1_radius*np.sin(np.deg2rad(angle)))))

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
        u[1] = 1.0
    elif sel == 5:
        u[0] = 1.0
        u[1] = -1.0
    
    return u

# Euclidean distance function
def dist(x_,y_):
    return np.sqrt((y_[0]-x_[0])**2 + (y_[1]-x_[1])**2)

# Collition check function
def coll_check(p1_init, angle1_init, u1_sel, p2_init, angle2_init, u2_sel):
    p1_new = np.array([p1_init[0], p1_init[1]])
    angle1_new = angle1_init
    p2_new = np.array([p2_init[0], p2_init[1]])
    angle2_new = angle2_init

    new_delta = delta_t_control/sample
    u1_ = U_set(u1_sel)
    u2_ = U_set(u2_sel)

    all_points1 = []
    all_angles1 = []
    all_points2 = []
    all_angles2 = []

    for i in range(0, int(sample)):
        p1_new[0] = p1_new[0] + (u1_[0]+u1_[1])/2.0 * v_max * np.cos(np.deg2rad(angle1_new)) * new_delta
        p1_new[1] = p1_new[1] - (u1_[0]+u1_[1])/2.0 * v_max * np.sin(np.deg2rad(angle1_new)) * new_delta
        angle1_new = angle1_new + (u1_[1]-u1_[0])/2.0 * omega_max * new_delta

        p2_new[0] = p2_new[0] + (u2_[0]+u2_[1])/2.0 * v_max * np.cos(np.deg2rad(angle2_new)) * new_delta
        p2_new[1] = p2_new[1] - (u2_[0]+u2_[1])/2.0 * v_max * np.sin(np.deg2rad(angle2_new)) * new_delta
        angle2_new = angle2_new + (u2_[1]-u2_[0])/2.0 * omega_max * new_delta

        p1_new_ = np.array([int(round(p1_new[0])), int(round(p1_new[1]))])
        p2_new_ = np.array([int(round(p2_new[0])), int(round(p2_new[1]))])

        all_points1.append(p1_new)
        all_angles1.append(angle1_new)
        all_points2.append(p2_new)
        all_angles2.append(angle2_new)

        _dist =  dist(p1_new, p2_new)
        if _dist > 1.5*ddr1_diam or _dist < ddr1_diam:
            return True, 0, 0

        if 0<=p1_new_[0]<Length and 0<=p1_new_[1]<Width and 0<=p2_new_[0]<Length and 0<=p2_new_[1]<Width:
            if screen.get_at(p1_new_) == black or free_sourranding(p1_new) == False or screen.get_at(p2_new_) == black or free_sourranding(p2_new) == False:
                return True, 0, 0
                
                    
    
    return False, all_points1, all_angles1, all_points2, all_angles2


## RRT Algorithm
def RRT(x1_init, angle1_init, x2_init, angle2_init, x_goal):
    goal_reached = False
    T = []

    x_new = []
    x_new.append(x1_init)
    x_new.append(angle1_init)
    x_new.append(x2_init)
    x_new.append(angle2_init)
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

        # Random controls
        rand_u1 = random.randint(1, 5)
        rand_u2 = random.randint(1, 5)

        # Check if this control is valid
        if x_near == 0:
            test = coll_check(T[x_near][0], T[x_near][1], rand_u1, T[x_near][2], T[x_near][3], rand_u2)
        else:
            test = coll_check(T[x_near][0][-1], T[x_near][1][-1], rand_u1, T[x_near][2][-1], T[x_near][3][-1], rand_u2)
            
        if test[0] == False:
            x_new = []
            x_new.append(test[1])
            x_new.append(test[2])
            x_new.append(test[3])
            x_new.append(test[4])
            x_new.append(rand_u1)
            x_new.append(rand_u2)
            x_new.append(x_near)

            T.append(x_new)

            if dist(test[1][-1], x_goal) < epsilon or dist(test[3][-1], x_goal) < epsilon:
                goal_reached = True
                break

    traj1 = []
    angles1 = []
    traj2 = []
    angles2 = []
    if goal_reached:
        traj1.append(T[-1][0])
        angles1.append(T[-1][1])
        traj2.append(T[-1][2])
        angles2.append(T[-1][3])
        parent = T[-1][6]
        
        while parent != 0:
            traj1.append(T[parent][0])
            angles1.append(T[parent][1])
            traj2.append(T[parent][2])
            angles2.append(T[parent][3])
            parent = T[parent][6]
    
    traj1.reverse()
    angles1.reverse()
    traj2.reverse()
    angles2.reverse()

    print('Total nodes used', i)
    return goal_reached, traj1, angles1, traj2, angles2



# Game loop
running = True
state = 0
valid = valid2 = valid3 = False
goal = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_pressed()[0] and state == 0 and valid == True:
            ddr1_pos = np.asarray(pygame.mouse.get_pos())
            ddr1_pos = ddr1_pos.astype('float64')
            print('x_init_1', ddr1_pos)
            state = 1
        if pygame.mouse.get_pressed()[0] and state == 1 and valid3 == True:
            ddr2_pos = np.asarray(pygame.mouse.get_pos())
            ddr2_pos = ddr2_pos.astype('float64')
            print('x_init_2', ddr2_pos)
            state = 2
        if pygame.mouse.get_pressed()[0] and state == 2 and valid2 == True:
            pygame.draw.circle(screen, red, goal, ddr1_radius)
            print('x_goal', goal)
            state = 3
        if pygame.key.get_pressed()[273] == 1 and state == 0:
            ddr1_angle = 90.0
        if pygame.key.get_pressed()[274] == 1 and state == 0:
            ddr1_angle = 270.0
        if pygame.key.get_pressed()[275] == 1 and state == 0:
            ddr1_angle = 0.0
        if pygame.key.get_pressed()[276] == 1 and state == 0:
            ddr1_angle = 180.0

        if pygame.key.get_pressed()[273] == 1 and state == 1:
            ddr2_angle = 90.0
        if pygame.key.get_pressed()[274] == 1 and state == 1:
            ddr2_angle = 270.0
        if pygame.key.get_pressed()[275] == 1 and state == 1:
            ddr2_angle = 0.0
        if pygame.key.get_pressed()[276] == 1 and state == 1:
            ddr2_angle = 180.0


    # Environment
    screen.fill(white)
    draw_Obstacles(obs)
    
    if state == 0:  # Place ship1
        p = np.asarray(pygame.mouse.get_pos())
        valid = free_sourranding(p)
            
        if valid:
            draw_DDR_1(p, ddr1_angle)
    
    elif state == 1:    # Place ship2
        p = np.asarray(pygame.mouse.get_pos())
        valid3 = free_sourranding(p)

        min_dist =  dist(ddr1_pos, p)
            
        if valid3 and min_dist < 1.5*ddr1_diam and min_dist > ddr1_diam:
            draw_DDR_2(p, ddr2_angle)

        draw_DDR_1(ddr1_pos, ddr1_angle)

    elif state == 2:    # Place goal
        goal = np.asarray(pygame.mouse.get_pos())
        valid2 = free_sourranding(goal)
            
        if valid2:
            pygame.draw.circle(screen, red, goal, ddr1_radius)
        
        draw_DDR_1(ddr1_pos, ddr1_angle)
        draw_DDR_2(ddr2_pos, ddr2_angle)

    elif state == 3:
        start_time = time.time()
        reached, Traj1, Ang1, Traj2, Ang2 = RRT(ddr1_pos, ddr1_angle, ddr2_pos, ddr2_angle, goal)
        elapsed_time = time.time() - start_time

        print('Time to compute trajectorie', elapsed_time)

        if reached:
            start_time = time.time()
            for k in range(0,len(Traj2)):
                for j in range(0,int(sample)):
                    ddr1_pos = Traj1[k][j]
                    ddr1_angle = Ang1[k][j]
                    ddr2_pos = Traj2[k][j]
                    ddr2_angle = Ang2[k][j]

                    screen.fill(white)
                    draw_Obstacles(obs)

                    pygame.draw.circle(screen, red, goal, ddr1_radius)
                    draw_DDR_1(ddr1_pos, ddr1_angle)
                    draw_DDR_2(ddr2_pos, ddr2_angle)

                    pygame.display.update()
                    pygame.time.delay(int(delta_t_control*1000/sample))
            
            elapsed_time = time.time() - start_time
            print('Time to get to goal', elapsed_time)

            print('Goal reached')

        else:
            print('Goal not reached')

        state = 4
    elif state == 4:
        pygame.draw.circle(screen, red, goal, ddr1_radius)
        draw_DDR_1(ddr1_pos, ddr1_angle)
        draw_DDR_2(ddr2_pos, ddr2_angle)

    
    pygame.display.update()
    pygame.time.delay(int(delta_t))
