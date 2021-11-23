import pygame
import time
import numpy as np
import random


# Colors
white = (255,255,255)
black = (0,0,0)
purple = (196,18,211)
light_green = (100,200,0)
darker_green = (0,130,0)
dark_blue = (0,80,165)
light_blue = (0,0,255)
red = (255,0,0)


## Initialize pygame
pygame.init()
pygame.display.set_caption('RRT*')

Length = 800
Width = 600
screen = pygame.display.set_mode((Length,Width))
screen.fill(white)


# Euclidean distance function
def dist(x,y):
    return np.sqrt((y[0]-x[0])**2 + (y[1]-x[1])**2)


# Collition check function
def coll_check(x1, x2):
    ratio = 0.0
    samples = 10

    for i in range(1,samples):
        ratio += 1.0/samples 
        x = x1[0] + int(ratio*(x2[0] - x1[0]))
        y = x1[1] + int(ratio*(x2[1] - x1[1]))

        if screen.get_at((x, y)) == black:
            return True

    return False



# RRT* algorithm
def RRT_star(x_init, x_goal, step_max, max_neighborhood, n, epsilon):
    T = []

    x_new = []
    x_new.append(x_init)
    x_new.append(0.0)
    x_new.append(0)

    T.append(x_new)

    for i in range(0,n):
        x = random.randint(0,Length - 1)
        y = random.randint(0,Width - 1)
        x_rand = np.array([x, y])

        # find nearest node
        d_node = float('inf')
        for k in range(len(T)):
            if dist(x_rand, T[k][0]) < d_node:
                x_near = k
                d_node = dist(x_rand, T[k][0])
        
        # Constrain edge to step_max
        d_near = dist(x_rand, T[x_near][0])
        if d_near > step_max:
            ratio = step_max/d_near
            x = T[x_near][0][0] + int(ratio*(x_rand[0] - T[x_near][0][0]))
            y = T[x_near][0][1] + int(ratio*(x_rand[1] - T[x_near][0][1]))
            x_rand = np.array([x, y])
            
        
        # Check if x_rand is in free space
        if screen.get_at(x_rand) != black:
            
            # Look for nodes in neigborhood
            Neighborhood = []
            for k in range(len(T)):
                if dist(x_rand, T[k][0]) < max_neighborhood:
                    node_Neighborhhod = []
                    node_Neighborhhod.append(T[k])
                    node_Neighborhhod.append(k)

                    Neighborhood.append(node_Neighborhhod)
            
            # Order Neighborhood by cost
            Neighborhood.sort(key = lambda t: t[1])

            # Find nearest node by cost
            for k in Neighborhood:
                if coll_check(k[0][0], x_rand) == False:
                    x_new = []
                    x_new.append(x_rand)
                    d = dist(x_rand, k[0][0])
                    x_new.append(d + k[0][1])
                    x_new.append(k[1])

                    T.append(x_new)

                    if dist(x_rand, x_goal) < epsilon: # If the goal is reached
                        return T, True, i
                    
                    ## Rewire
                    for k in Neighborhood:
                        d = dist(x_rand, k[0][0])
                        if x_new[1] + d < k[0][1] and coll_check(x_rand, k[0][0]) == False:
                            k[0][2] = len(T) - 1
                            k[0][1] = x_new[1] + d
                    ######
                    break
    
    return T, False, i



# game loop
running = True
state = 0
while running:
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_pressed()[0] and state == 0:
            mouse_pos = pygame.mouse.get_pos()
            pygame.draw.circle(screen, black, mouse_pos, 10)

        if pygame.mouse.get_pressed()[0] and 0 < state < 100:
            mouse_pos = pygame.mouse.get_pos()
            pygame.draw.circle(screen, purple, mouse_pos, 8)

            x_init = np.asarray(mouse_pos)

            state = 101
        if pygame.mouse.get_pressed()[2] and 100 < state < 200:
            mouse_pos = pygame.mouse.get_pos()
            pygame.draw.circle(screen, red, mouse_pos, 20)

            x_goal = np.asarray(mouse_pos)

            state = 300
            pygame.display.update()
        if pygame.key.get_pressed()[13] == 1:
            state += 1

    if state > 200:
        start_time = time.time()
        T, goal_reached, nodes = RRT_star(x_init,x_goal, 20.0, 60.0, 10000, 30.0)
        elapsed_time = time.time() - start_time

        print('Time needed to biuld the Tree ', elapsed_time, ' seconds')
        state = -1

        # Plot tree
        node_radius = 5
        line_wide = 2
        for k in range(1,len(T)):
            pygame.draw.line(screen, light_green, T[k][0], T[T[k][2]][0], line_wide)
            pygame.draw.circle(screen, darker_green, T[k][0], node_radius)
        
        pygame.draw.circle(screen, purple, x_init, 8)     # Draw x_init point again

        # Plot trayectorie fron x_init to x_goal
        if goal_reached:
            print('Goal Reached')
            print('Cost to the goal ', T[-1][1])
            print('Nodes expantion ', nodes)

            index = T[-1][2]
            node = T[index][0]
            pygame.draw.line(screen, light_blue, x_goal, node, line_wide)
            pygame.draw.circle(screen, dark_blue, node, node_radius)

            index = T[index][2]
            while index != 0:
                pygame.draw.line(screen, light_blue, node, T[index][0], line_wide)
                pygame.draw.circle(screen, dark_blue, T[index][0], node_radius)

                node = T[index][0]
                index = T[index][2]
            
            pygame.draw.line(screen, light_blue, node, x_init, line_wide)
        else:
            print('Goal NOT Reached')

    pygame.display.update()