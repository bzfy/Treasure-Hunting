#!/usr/bin/python3
# ^^ note the python directive on the first line
# COMP3411/9814 agent initiation file 
# requires the host to be running before the agent
# typical initiation would be (file in working directory, port = 31415)
#        python3 agent.py -p 31415
# created by Leo Hoare
# with slight modifications by Alan Blair

'''
algorithm: bfs, A star, A star with waypoint

Map this class to maintain map space has been obtained

get action logic: 
    1:If A treasure is found on the map, use the A* algorithm with path points to find the treasure and go home, 
    where the treasure is the path point and the home is the destination point

    2:If this path is not found, exploring the boundary without cost means exploring the land as far as possible with bfs, 
    finding the boundary, and using the normal A* algorithm to reach it

    3:If there are no cost boundaries, traverse the map looking for items k, a, d, and 
    if there are items use the A* algorithm to find if there is a viable path

    4:If there is no viable path but there is a raft, look for water and other land boundaries, 
    and if you are on a raft, explore as much water space as possible first, then explore land space

    5:If the exploration is completed, use the tools to explore the unknown space, 
    the first use the key to unlock the door, 
    the second use the axe to chop the tree, 
    the third use the dynamite to bomb the wall, 
    in which need to confirm the location of the use of tools, 
    but also need to adjust the direction of the use of tools.


'''


import sys
import socket
import heapq
from collections import deque

# declaring visible grid to agent
view = [['' for _ in range(5)] for _ in range(5)]


class Map:
    def __init__(self):
        self.grid = {}  # map space
        self.current_pos = (0, 0)  # play positon
        self.initialized = False  # nitialize
        self.star = (0,0) # home
    def update_map(self, new_grid, direction = None):

        # print(new_grid) # test
        # initializ map
        if not self.initialized:
          
            for i in range(5):
                for j in range(5):
                    self.grid[(i - 2, j - 2)] = new_grid[i][j]
            # print('grid',self.grid) # test
            
            self.initialized = True
            return

        x, y = self.current_pos

        # core logical mapping from 5*5 to big map
        if direction == 'up':
            if self.grid.get((x - 1, y)) in ['*', '-', 'T']:
                return
            x -= 1
            for i in range(5):
                for j in range(5):
                    self.grid[(x + i - 2, y + j - 2)] = new_grid[i][j]
            self.grid[(x,y)] = '^'
        elif direction == 'down':
            if self.grid.get((x + 1, y)) in ['*', '-', 'T']:
                return
            x += 1
            for i in range(5):
                for j in range(5):
                    self.grid[(x - i + 2, y - j + 2)] = new_grid[i][j]
            self.grid[(x,y)] = 'v'
        elif direction == 'left':
            if self.grid.get((x, y - 1)) in ['*', '-', 'T']:
                return
            y -= 1
            for i in range(5):
                for j in range(5):
                    self.grid[(x - j + 2, y + i - 2)] = new_grid[i][j]
            self.grid[(x,y)] = '<'
        elif direction == 'right':
            if self.grid.get((x, y + 1)) in ['*', '-', 'T']:
                return
            y += 1
            for i in range(5):
                for j in range(5):
                    self.grid[(x + j - 2, y - i + 2)] = new_grid[i][j]
            self.grid[(x,y)] = '>'
        elif direction is None:
            # change directiono no change position
            orient = direction_global  # current direction
            if orient == 'up':
                for i in range(5):
                    for j in range(5):
                        self.grid[(x + i - 2, y + j - 2)] = new_grid[i][j]
                self.grid[(x, y)] = '^'
            elif orient == 'down':
                for i in range(5):
                    for j in range(5):
                        self.grid[(x - i + 2, y - j + 2)] = new_grid[i][j]
                self.grid[(x, y)] = 'v'
            elif orient == 'left':
                for i in range(5):
                    for j in range(5):
                        self.grid[(x - j + 2, y + i - 2)] = new_grid[i][j]
                self.grid[(x, y)] = '<'
            elif orient == 'right':
                for i in range(5):
                    for j in range(5):
                        self.grid[(x + j - 2, y - i + 2)] = new_grid[i][j]
                self.grid[(x, y)] = '>'

        # home
        # self.grid[self.star] = 'S'
        
        # print('grid',self.grid) # test
        # updata current position
        self.current_pos = (x, y)
    def display(self):

        if not self.grid:
            # print('map is None')
            return
        
        # frontier
        min_x = min(x for x, y in self.grid)
        max_x = max(x for x, y in self.grid)
        min_y = min(y for x, y in self.grid)
        max_y = max(y for x, y in self.grid)

        # print
        for x in range(min_x, max_x + 1):
            row = [self.grid.get((x, y), '?') for y in range(min_y, max_y + 1)]
            print(''.join(row))

# utility
def is_frontier(cell):
    x, y = cell
    # (5*5)
    for i in range(-2, 3):      # -2, -1, 0, 1, 2
        for j in range(-2, 3):
            nx, ny = x + i, y + j
            if (nx, ny) not in game_map.grid:
                # yes
                return True
            if game_map.grid.get((nx, ny)) == '?':
                # yes '?'
                return True
    # no
    return False
def astar_path_with_waypoint(start_state, goal_pos, treasure_pos):
    '''
    start_state = (game_map.current_pos[0], 
                game_map.current_pos[1],
                orient_map[direction_global], 
                have_key, 
                have_axe,
                num_dynamites_hold, 
                have_raft, 
                on_raft)
    '''
    def heuristic(x, y, have_treasure):
        # heuristic value is (current positon to treasure postion) + (treasure positon to home)
        if not have_treasure:
            return abs(x - treasure_pos[0]) + abs(y - treasure_pos[1]) + abs(treasure_pos[0] - goal_pos[0]) + abs(treasure_pos[1] - goal_pos[1])
        else:
            return abs(x - goal_pos[0]) + abs(y - goal_pos[1])
    
    # Extend state: have_treasure flag initial False
    start = tuple(list(start_state) + [False]) + (frozenset(),)
    open_heap = []
    init_h = heuristic(start_state[0], start_state[1], False)
    heapq.heappush(open_heap, (init_h, 0, start))
    
    parent = {start: None}
    parent_action = {start: None}
    cost_so_far = {start: 0}
    goal_state = None
    # closed_set = set()


    while open_heap:
        # print('1')
        f, g, state = heapq.heappop(open_heap)
        if g > cost_so_far.get(state, float('inf')):
            continue

        ######################
        # if state in closed_set:
        #     continue
        # closed_set.add(state)
        ######################


        x, y, orient, have_key, have_axe, bombs, have_raft, on_raft, have_treasure, mods = state
        

        if (x, y) == goal_pos and have_treasure:
            goal_state = state
            break


        # left
        new_orient = (orient - 1) % 4
        new_state = (x, y, new_orient, have_key, have_axe, bombs, have_raft, on_raft, have_treasure, mods)
        new_cost = g + 1
        if new_cost < cost_so_far.get(new_state, float('inf')):
            cost_so_far[new_state] = new_cost
            parent[new_state] = state
            parent_action[new_state] = 'L'
            heapq.heappush(open_heap, (new_cost + heuristic(x, y, have_treasure), new_cost, new_state))


        # F postion
        if orient == 0:
            nx, ny = x - 1, y
        elif orient == 1:
            nx, ny = x, y + 1
        elif orient == 2:
            nx, ny = x + 1, y
        else:
            nx, ny = x, y - 1
        
        cell = game_map.grid.get((nx, ny), '?')
        if (nx, ny) in mods:
            cell = ' '

        if cell in ['.']:
            continue
        
        # F
        if cell == '~':
            if on_raft or have_raft:
                new_on_raft = True
                new_have_raft = have_raft
                if not on_raft and have_raft:
                    new_on_raft = True
                    new_have_raft = False
                new_state = (nx, ny, orient, have_key, have_axe, bombs, new_have_raft, new_on_raft, have_treasure, mods)
                step_cost = 5 if not on_raft else 1 # from land into water cost 5，mover in water cost 1
                new_cost = g + step_cost
                if new_cost < cost_so_far.get(new_state, float('inf')):
                    cost_so_far[new_state] = new_cost
                    parent[new_state] = state
                    parent_action[new_state] = 'F'
                    heapq.heappush(open_heap, (new_cost + heuristic(nx, ny, have_treasure), new_cost, new_state))
        elif cell in [' ', 'k', 'a', 'd', '$']:
            new_on_raft = on_raft
            new_have_raft = have_raft
            if on_raft and cell != '~':
                new_on_raft = False
                new_have_raft = False
            new_have_key = have_key
            new_have_axe = have_axe
            new_bombs = bombs
            new_have_treasure = have_treasure
            new_mods = mods
            if cell == 'k' and (nx, ny) not in mods:
                new_have_key = True
                new_mods = mods.union({(nx, ny)})
            elif cell == 'a' and (nx, ny) not in mods:
                new_have_axe = True
                new_mods = mods.union({(nx, ny)})
            elif cell == 'd' and (nx, ny) not in mods:
                new_bombs = bombs + 1
                new_mods = mods.union({(nx, ny)})
            elif cell == '$' and (nx, ny) not in mods:
                new_have_treasure = True
                new_mods = mods.union({(nx, ny)})
            step_cost = 5 if on_raft else 1 # from water into land cost 5，mover in land cost 1
            new_cost = g + step_cost
            new_state = (nx, ny, orient, new_have_key, new_have_axe, new_bombs, new_have_raft, new_on_raft, new_have_treasure, new_mods)
            if new_cost < cost_so_far.get(new_state, float('inf')):
                cost_so_far[new_state] = new_cost
                parent[new_state] = state
                parent_action[new_state] = 'F'
                heapq.heappush(open_heap, (new_cost + heuristic(nx, ny, new_have_treasure), new_cost, new_state))
        
        # unlock door
        cell_door = game_map.grid.get((nx, ny), '?')
        if (nx, ny) in mods:
            cell_door = ' '
        if cell_door == '-' and have_key:
            new_mods = mods.union({(nx, ny)})
            new_state = (x, y, orient, have_key, have_axe, bombs, have_raft, on_raft, have_treasure, new_mods)
            new_cost = g + 1
            if new_cost < cost_so_far.get(new_state, float('inf')):
                cost_so_far[new_state] = new_cost
                parent[new_state] = state
                parent_action[new_state] = 'U'
                heapq.heappush(open_heap, (new_cost + heuristic(x, y, have_treasure), new_cost, new_state))
        
        # use dynamite
        cell_wall = game_map.grid.get((nx, ny), '?')
        if (nx, ny) in mods:
            cell_wall = ' '
        if bombs > 0 and cell_wall == '*':
            new_mods = mods.union({(nx, ny)})
            new_state = (x, y, orient, have_key, have_axe, bombs - 1, have_raft, on_raft, have_treasure, new_mods)
            bomb_cost = 5 if not on_raft else 8
            new_cost = g + bomb_cost
            if new_cost < cost_so_far.get(new_state, float('inf')):
                cost_so_far[new_state] = new_cost
                parent[new_state] = state
                parent_action[new_state] = 'B'
                heapq.heappush(open_heap, (new_cost + heuristic(x, y, have_treasure), new_cost, new_state))
        
        # left
        # new_orient = (orient - 1) % 4
        # new_state = (x, y, new_orient, have_key, have_axe, bombs, have_raft, on_raft, have_treasure, mods)
        # new_cost = g + 1
        # if new_cost < cost_so_far.get(new_state, float('inf')):
        #     cost_so_far[new_state] = new_cost
        #     parent[new_state] = state
        #     parent_action[new_state] = 'L'
        #     heapq.heappush(open_heap, (new_cost + heuristic(x, y, have_treasure), new_cost, new_state))
        
        # right
        # new_orient = (orient + 1) % 4
        # new_state = (x, y, new_orient, have_key, have_axe, bombs, have_raft, on_raft, have_treasure, mods)
        # new_cost = g + 1
        # if new_cost < cost_so_far.get(new_state, float('inf')):
        #     cost_so_far[new_state] = new_cost
        #     parent[new_state] = state
        #     parent_action[new_state] = 'R'
        #     heapq.heappush(open_heap, (new_cost + heuristic(x, y, have_treasure), new_cost, new_state))
        
        # chop tree
        cell_tree = game_map.grid.get((nx, ny), '?')
        if (nx, ny) in mods:
            cell_tree = ' '
        if cell_tree == 'T' and have_axe:
            new_have_raft = True
            new_mods = mods.union({(nx, ny)})
            new_state = (x, y, orient, have_key, have_axe, bombs, new_have_raft, on_raft, have_treasure, new_mods)
            new_cost = g + 2
            if new_cost < cost_so_far.get(new_state, float('inf')):
                cost_so_far[new_state] = new_cost
                parent[new_state] = state
                parent_action[new_state] = 'C'
                heapq.heappush(open_heap, (new_cost + heuristic(x, y, have_treasure), new_cost, new_state))
    
    if goal_state is None:
        return None
    
    # reverse actionsss
    actions = []
    s = goal_state
    while parent_action[s] is not None:
        actions.append(parent_action[s])
        s = parent[s]
    actions.reverse()
    return actions


def astar_path(start_state, goal_pos):
    '''
    start_state = (game_map.current_pos[0], 
                game_map.current_pos[1],
                orient_map[direction_global], 
                have_key, 
                have_axe,
                num_dynamites_hold, 
                have_raft, 
                on_raft)
    '''
    def heuristic(x, y):
        return abs(x - goal_pos[0]) + abs(y - goal_pos[1])

    # extned mods state, to save every change in map in every wayss
    start = tuple(start_state) + (frozenset(),)
    open_heap = []
    heapq.heappush(open_heap, (heuristic(start_state[0], start_state[1]), 0, start))
    
    parent = {start: None}
    parent_action = {start: None}
    cost_so_far = {start: 0}
    goal_state = None
    closed_set = set()



    while open_heap:

        f, g, state = heapq.heappop(open_heap)
        if g > cost_so_far.get(state, float('inf')):
            continue


        ######################
        if state in closed_set:
            continue
        closed_set.add(state)
        ######################

        x, y, orient, have_key, have_axe, bombs, have_raft, on_raft, mods = state
        if (x, y) == goal_pos:
            goal_state = state
            break
        # if game_map.grid.get(goal_pos, '?') in ['d'] and have_axe:
            # print(open_heap)




        # # left
        # new_orient = (orient - 1) % 4
        # new_state = (x, y, new_orient, have_key, have_axe, bombs, have_raft, on_raft, mods)
        # new_cost = g + 1
        # if new_cost < cost_so_far.get(new_state, float('inf')):
        #     cost_so_far[new_state] = new_cost
        #     parent[new_state] = state
        #     parent_action[new_state] = 'L'
        #     heapq.heappush(open_heap, (new_cost + heuristic(x, y), new_cost, new_state))   


        # F postion
        if orient == 0:
            nx, ny = x - 1, y
        elif orient == 1:
            nx, ny = x, y + 1
        elif orient == 2:
            nx, ny = x + 1, y
        else:
            nx, ny = x, y - 1

        cell = game_map.grid.get((nx, ny), '?')
        #########################################################################
        # if (nx,ny) == (4,13):
        #     print('-'*1000)
        # 
        # if (nx,ny) == (4,13):
        #     print('cell',cell)
        # if (nx, ny) in mods:
        #     cell = ' '
        # if (nx,ny) == (4,13):
        #     print('cell',cell)       
        #     print('state',state)         
        # # 获取目标格内容
        # if (nx,ny) == (5,13):
        #     print('-'*1000)
        # cell = game_map.grid.get((nx, ny), '?')
        # if (nx,ny) == (5,13):
        #     print('cell',cell)
        # if (nx, ny) in mods:
        #     cell = ' '
        # if (nx,ny) == (5,13):
        #     print('cell',cell)       
        #     print('state',state)       
        #############################################################################

        # 1: F
        if cell in ['.','?']:
            continue

        if cell == '~':
            if on_raft or have_raft:
                new_on_raft = True
                new_have_raft = have_raft
                if not on_raft and have_raft:
                    new_on_raft = True
                    new_have_raft = False
                new_state = (nx, ny, orient, have_key, have_axe, bombs, new_have_raft, new_on_raft, mods)
                step_cost = 5 if not on_raft else 1  # from land into water cost 5，mover in water cost 1
                new_cost = g + step_cost
                # if (nx,ny) == (5,12):
                #     print(f'{"*"*1000}')
                #     print(new_cost)
                #     print(cost_so_far.get(new_state, float('inf')))
                # if (nx,ny) == (5,13):
                #     print(f'{"+"*1000}')
                #     print(new_cost)
                if new_cost < cost_so_far.get(new_state, float('inf')):
                    cost_so_far[new_state] = new_cost
                    parent[new_state] = state
                    parent_action[new_state] = 'F'
                    heapq.heappush(open_heap, (new_cost + heuristic(nx, ny), new_cost, new_state))
        elif cell in [' ', 'k', 'a', 'd', '$']:
            new_on_raft = on_raft
            new_have_raft = have_raft
            if on_raft and cell != '~':
                new_on_raft = False
                new_have_raft = False
            new_have_key = have_key
            new_have_axe = have_axe
            new_bombs = bombs
            new_mods = mods
            # Pick up items only once
            if cell == 'k' and (nx, ny) not in mods:
                new_have_key = True
                new_mods = mods.union({(nx, ny)})
            elif cell == 'a' and (nx, ny) not in mods:
                new_have_axe = True
                new_mods = mods.union({(nx, ny)})
            elif cell == 'd' and (nx, ny) not in mods:
                new_bombs = bombs + 1
                new_mods = mods.union({(nx, ny)})
            step_cost = 5 if on_raft else 1  # from water into land cost 5，mover in land cost 1
            new_cost = g + step_cost
            new_state = (nx, ny, orient, new_have_key, have_axe, new_bombs, new_have_raft, new_on_raft, new_mods)
            if new_cost < cost_so_far.get(new_state, float('inf')):
                cost_so_far[new_state] = new_cost
                parent[new_state] = state
                parent_action[new_state] = 'F'
                heapq.heappush(open_heap, (new_cost + heuristic(nx, ny), new_cost, new_state))


        cell_tree = game_map.grid.get((nx, ny), '?')
        if (nx, ny) in mods:
            cell_tree = ' '
        if cell_tree == 'T' and have_axe:


            ############################################
            # print('chop tree')
            # print('mods', mods)
            # print('nx, ny', nx, ny)
            #############################################


            new_have_raft = True
            new_mods = mods.union({(nx, ny)})


            #####################################
            # print('newmods',new_mods)
            ###########################


            new_state = (x, y, orient, have_key, have_axe, bombs, new_have_raft, on_raft, new_mods)
            # chop tree cost 2
            new_cost = g + 2


            #####################################
            # print(new_cost,'',cost_so_far.get(new_state, float('inf')))
            #####################################


            if new_cost < cost_so_far.get(new_state, float('inf')):
                cost_so_far[new_state] = new_cost
                parent[new_state] = state
                parent_action[new_state] = 'C'
                heapq.heappush(open_heap, (new_cost + heuristic(nx, ny), new_cost, new_state))

        # unlock door
        cell_door = game_map.grid.get((nx, ny), '?')
        if (nx, ny) in mods:
            cell_door = ' '
        if cell_door == '-' and have_key:
            new_mods = mods.union({(nx, ny)})
            new_state = (x, y, orient, have_key, have_axe, bombs, have_raft, on_raft, new_mods)
            new_cost = g + 1
            if new_cost < cost_so_far.get(new_state, float('inf')):
                cost_so_far[new_state] = new_cost
                parent[new_state] = state
                parent_action[new_state] = 'U'
                heapq.heappush(open_heap, (new_cost + heuristic(x, y), new_cost, new_state))
        
        # use dynamite
        cell_wall = game_map.grid.get((nx, ny), '?')
        if (nx, ny) in mods:
            cell_wall = ' '
        if bombs > 0 and cell_wall == '*':
            new_mods = mods.union({(nx, ny)})
            new_state = (x, y, orient, have_key, have_axe, bombs - 1, have_raft, on_raft, new_mods)
            # land bomb cost 5
            bomb_cost = 5 if not on_raft else 8
            new_cost = g + bomb_cost
            if new_cost < cost_so_far.get(new_state, float('inf')):
                cost_so_far[new_state] = new_cost
                parent[new_state] = state
                parent_action[new_state] = 'B'
                heapq.heappush(open_heap, (new_cost + heuristic(x, y), new_cost, new_state))
    
        # left
        # new_orient = (orient - 1) % 4
        # new_state = (x, y, new_orient, have_key, have_axe, bombs, have_raft, on_raft, mods)
        # new_cost = g + 1
        # if new_cost < cost_so_far.get(new_state, float('inf')):
        #     cost_so_far[new_state] = new_cost
        #     parent[new_state] = state
        #     parent_action[new_state] = 'L'
        #     heapq.heappush(open_heap, (new_cost + heuristic(x, y), new_cost, new_state))
        
        # right
        new_orient = (orient + 1) % 4
        new_state = (x, y, new_orient, have_key, have_axe, bombs, have_raft, on_raft, mods)
        new_cost = g + 1
        if new_cost < cost_so_far.get(new_state, float('inf')):
            cost_so_far[new_state] = new_cost
            parent[new_state] = state
            parent_action[new_state] = 'R'
            heapq.heappush(open_heap, (new_cost + heuristic(x, y), new_cost, new_state))
        



    ######################
    # plt.ioff()
    # plt.show()
    ######################

    if goal_state is None:
        return None
    
    # reverse actionsss
    actions = []
    s = goal_state
    while parent_action[s] is not None:
        actions.append(parent_action[s])
        s = parent[s]
    actions.reverse()
    return actions




# global state

game_map = Map()
direction_deque = deque(['up', 'right', 'down', 'left'])  # cycle direction deque
direction_global = direction_deque[0]  # current direction


# global agent state
have_axe = False
have_key = False
have_treasure = False
have_raft = False
num_dynamites_hold = 0
on_raft = False
never_on_raft = True   

# plan action deque
plan_actions = deque()
action = None

# orient mapping 
orient_map = {'up': 0, 'right': 1, 'down': 2, 'left': 3}
orient_list = ['up', 'right', 'down', 'left']

# function to take get action from AI or user



def get_action(view_grid):

    global direction_deque, direction_global
    global action, plan_actions
    global have_axe, have_key, have_treasure, have_raft, num_dynamites_hold, on_raft
    global never_on_raft
    # first updata map
    if not game_map.initialized:
        game_map.update_map(view_grid, action)
        # print('----------11111111-----------')
        # game_map.display()

    else:
        # normal updata map
        if action in ['L', 'l']:
            direction_deque.rotate(1)
            direction_global = direction_deque[0]
            game_map.update_map(view_grid, None)
        elif action in ['R', 'r']:
            direction_deque.rotate(-1)
            direction_global = direction_deque[0]
            game_map.update_map(view_grid, None)
        elif action in ['F', 'f']:
            game_map.update_map(view_grid, direction_global)
        elif action in ['C', 'c', 'U', 'u', 'B', 'b']:
            game_map.update_map(view_grid, None)
    
    # Debug
    #########################################################################
    # game_map.display()
    # print('axe, key, treasure, raft, num_d, on_raft', have_axe, have_key, have_treasure, have_raft, num_dynamites_hold, on_raft)
    # print('action', action)
    # print('direction', direction_global)
    ##########################################################################

    # plan
    if plan_actions:
        next_act = plan_actions.popleft()
        # print("execute plan", next_act) # test
    else:
        next_act = None

        #####################################################################################
        # target1: have treasure go home (wrong)
        # 1) s4 s7 fail but s5 have a little problem 
        # if have_treasure and game_map.current_pos != game_map.star:
        # 2) s5 s7 fail skip target1 wrong target1
        # 2) only s7 fail, s4 spend long time
        if None:
            start_state = (game_map.current_pos[0], 
                           game_map.current_pos[1],
                           orient_map[direction_global], 
                           have_key, 
                           have_axe,
                           num_dynamites_hold, 
                           have_raft, 
                           on_raft)
            path_home = astar_path(start_state, game_map.star)

            if path_home:
                plan_actions = deque(path_home)
                next_act = plan_actions.popleft()

                #######################################
                # print('go home')
                # print('plan_actions', plan_actions)
                # print("execute plan", next_act) # test
                ########################################
            else:
                print('error: go home')
                next_act = 'L'  

        else:

            if next_act is None:
                # target2: no treasure find treasutre (wrong)
                treasure_pos = None
                for pos, val in game_map.grid.items():
                    if val == '$':
                        treasure_pos = pos
                        break
                
                if treasure_pos and not have_treasure:
                    # print('find dassssssssssssssssss')

                    start_state = (game_map.current_pos[0], 
                                   game_map.current_pos[1],
                                   orient_map[direction_global],
                                   have_key,
                                   have_axe,
                                   num_dynamites_hold, 
                                   have_raft, 
                                   on_raft)
                    #############
                    # print(start_state)
                    # print(game_map.star)
                    # print(treasure_pos)
                    #############

                    # 1) target 2 (wrong)
                    # path_to_treasure = astar_path(start_state, treasure_pos)
                    # 2) true target 1: find treasure and also go home both do (right)
                    path_to_treasure = astar_path_with_waypoint(start_state, game_map.star, treasure_pos)
                    
                    #############
                    # print(path_to_treasure)
                    #############

                    if path_to_treasure:
                        plan_actions = deque(path_to_treasure)
                        next_act = plan_actions.popleft()

                        ##################################################
                        # print('fand treasure', treasure_pos)
                        # print('plan_actions', plan_actions)
                        # print("execute plan", next_act) # test
                        ####################################################

                    else:
                        treasure_pos = None  # no way to treasure and go home both do

            # if next_act is None and never_on_raft:
            #     # frontier explore and never on raft before
            #     land_queue_1 = deque()
            #     start = game_map.current_pos
            #     visited = {start: 0}
            #     land_queue_1.append((start, 0))
            #     frontier_target = None

            #     while land_queue_1:
            #         cur, dist = land_queue_1.popleft()

            #         x, y = cur

            #         if is_frontier(cur):
            #             frontier_target = cur
            #             break

            #         for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            #             nxt = (x + dx, y + dy)
            #             if nxt in visited:
            #                 continue
            #             val = game_map.grid.get(nxt, '?')
            #             # only land
            #             if val in ['*', 'T', '-', '.', '?', '~']:
            #                 continue
            #             visited[nxt] = dist + 1
            #             land_queue_1.append((nxt, dist + 1))
            #     if frontier_target:
            #         start_state = (game_map.current_pos[0], 
            #                        game_map.current_pos[1],
            #                        orient_map[direction_global], 
            #                        have_key, 
            #                        have_axe,
            #                        num_dynamites_hold, 
            #                        have_raft, 
            #                        on_raft)
            #         # print('find frontier', frontier_target)
            #         path_to_frontier = astar_path(start_state, frontier_target)
            #         if path_to_frontier:
            #             plan_actions = deque(path_to_frontier)
            #             next_act = plan_actions.popleft()

            # target 3: find items
            if next_act is None:
                item_targets = []
                if not have_key:
                    item_targets += [(pos, 'k') for pos, v in game_map.grid.items() if v == 'k']
                if not have_axe:
                    item_targets += [(pos, 'a') for pos, v in game_map.grid.items() if v == 'a']
                item_targets += [(pos, 'd') for pos, v in game_map.grid.items() if v == 'd']
                shortest_path = None
                if item_targets:
                    start_state = (game_map.current_pos[0], 
                                   game_map.current_pos[1],
                                   orient_map[direction_global], 
                                   have_key, 
                                   have_axe,
                                   num_dynamites_hold, 
                                   have_raft, 
                                   on_raft)
                    # find most recent item
                    min_len = float('inf')
                    for pos, item_type in item_targets:
                        # print('find items', item_type)
                        path = astar_path(start_state, pos)

                        ############################
                        # if item_type == 'd' and not path:
                        #     print('dynamite no way')
                        # ###############################


                        if path and len(path) < min_len:
                            min_len = len(path)
                            shortest_path = path
                    if shortest_path:
                        plan_actions = deque(shortest_path)
                        next_act = plan_actions.popleft()

                        ##################################################
                        # print('find items', item_targets)
                        # print('plan_actions', plan_actions)
                        # print("execute plan", next_act) # test
                        ####################################################


            # target4: find frontier
            if next_act is None:
                # two deque water and land
                water_queue = deque()
                land_queue = deque()
                start = game_map.current_pos
                visited = {start: 0}

                # add position into deque base on current cell
                if on_raft:
                    water_queue.append((start, 0))
                else:
                    land_queue.append((start, 0))

                frontier_target = None
                # bfs
                while water_queue or land_queue:
                    # priority if have water 
                    if water_queue:
                        cur, dist = water_queue.popleft()
                    else:
                        cur, dist = land_queue.popleft()

                    x, y = cur

                    # core logical bfs
                    if is_frontier(cur):
                        frontier_target = cur
                        break
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nxt = (x + dx, y + dy)
                        if nxt in visited:
                            continue
                        val = game_map.grid.get(nxt, '?')
                        # barrier skip
                        if val in ['*', 'T', '-', '.', '?']:
                            continue
                        # no raft continue
                        if val == '~' and not have_raft:
                            continue

                        visited[nxt] = dist + 1
                        if val == '~':
                            water_queue.append((nxt, dist + 1))
                        else:
                            land_queue.append((nxt, dist + 1))
                if frontier_target:
                    start_state = (game_map.current_pos[0], 
                                   game_map.current_pos[1],
                                   orient_map[direction_global], 
                                   have_key, 
                                   have_axe,
                                   num_dynamites_hold, 
                                   have_raft, 
                                   on_raft)
                    # print('find frontier', frontier_target)
                    path_to_frontier = astar_path(start_state, frontier_target)
                    if path_to_frontier:
                        plan_actions = deque(path_to_frontier)
                        next_act = plan_actions.popleft()

                        ##################################################
                        ##################################################
                        # print('find frontier', frontier_target)
                        # print('plan_actions', plan_actions)
                        # print("execute plan", next_act) # 测试
                        ####################################################
                        ####################################################
                        
                # target5: if no frontier and no treasure and no iteams way, use iteams to find new frontier
                if next_act is None:
                    decided = False
                    # first unlock door
                    if have_key:
                        target_door = None
                        target_door_neighbor = None
                        for pos, v in game_map.grid.items():
                            if v == '-':
                                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                                    neigh = (pos[0] + dx, pos[1] + dy)
                                    if neigh in visited:
                                        target_door = pos
                                        target_door_neighbor = neigh
                        if target_door:
                            start_state = (game_map.current_pos[0],
                                           game_map.current_pos[1],
                                           orient_map[direction_global], 
                                           have_key, 
                                           have_axe,
                                           num_dynamites_hold, 
                                           have_raft, 
                                           on_raft)
                            path_to_door = astar_path(start_state, target_door_neighbor)
                            if path_to_door is not None:
                                door_plan = list(path_to_door)
                                # determin orientation
                                dx = target_door[0] - target_door_neighbor[0]
                                dy = target_door[1] - target_door_neighbor[1]
                                desired_orient = None
                                if dx == -1 and dy == 0:
                                    desired_orient = 0  # up
                                elif dx == 1 and dy == 0:
                                    desired_orient = 2  # down
                                elif dx == 0 and dy == -1:
                                    desired_orient = 3  # left
                                elif dx == 0 and dy == 1:
                                    desired_orient = 1  # right
                                # reach door neighbor orientation
                                orient_idx = orient_map[direction_global]
                                for act in door_plan:
                                    if act in ['L', 'l']:
                                        orient_idx = (orient_idx - 1) % 4
                                    elif act in ['R', 'r']:
                                        orient_idx = (orient_idx + 1) % 4
                                # change orientation before unlock
                                if desired_orient is not None:
                                    diff = (desired_orient - orient_idx) % 4
                                    if diff == 3:
                                        door_plan.append('L')
                                    elif diff == 2:
                                        door_plan.append('L'); door_plan.append('L')
                                    elif diff == 1:
                                        door_plan.append('R')
                                door_plan.append('U')
                                plan_actions = deque(door_plan)
                                next_act = plan_actions.popleft()


                                ##################################################
                                # print('unlock dooor', target_door)
                                # print('plan_actions', plan_actions)
                                # print("execute plan", next_act) # test
                                ####################################################

                                decided = True
                    # second chop tree
                    # if not decided and have_axe and not have_raft:
                    if not decided and have_axe and not have_raft and any(val == '~' for val in game_map.grid.values()):
                        min_dist = float('inf')
                        target_tree = None
                        target_tree_neighbor = None
                        for pos, v in game_map.grid.items():
                            if v == 'T':
                                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                                    neigh = (pos[0] + dx, pos[1] + dy)
                                    if neigh in visited:
                                        dist = visited[neigh]
                                        if dist < min_dist:
                                            min_dist = dist
                                            target_tree = pos
                                            target_tree_neighbor = neigh
                        if target_tree:
                            start_state = (game_map.current_pos[0], 
                                           game_map.current_pos[1],
                                           orient_map[direction_global], 
                                           have_key, 
                                           have_axe,
                                           num_dynamites_hold, 
                                           have_raft, 
                                           on_raft)
                            # print('tree')
                            path_to_tree = astar_path(start_state, target_tree_neighbor)
                            if path_to_tree is not None:
                                tree_plan = list(path_to_tree)
                                # determin orientation
                                tx, ty = target_tree
                                nx, ny = target_tree_neighbor
                                dx = tx - nx
                                dy = ty - ny
                                desired_orient = None
                                if dx == -1 and dy == 0:
                                    desired_orient = 0  # up
                                elif dx == 1 and dy == 0:
                                    desired_orient = 2  # down
                                elif dx == 0 and dy == -1:
                                    desired_orient = 3  # left
                                elif dx == 0 and dy == 1:
                                    desired_orient = 1  # right
                                # reach door neighbor orientation
                                orient_idx = orient_map[direction_global]
                                for act in tree_plan:
                                    if act in ['L', 'l']:
                                        orient_idx = (orient_idx - 1) % 4
                                    elif act in ['R', 'r']:
                                        orient_idx = (orient_idx + 1) % 4
                                # change orientation before chop
                                if desired_orient is not None:
                                    diff = (desired_orient - orient_idx) % 4
                                    if diff == 3:
                                        tree_plan.append('L')
                                    elif diff == 2:
                                        tree_plan.append('L'); tree_plan.append('L')
                                    elif diff == 1:
                                        tree_plan.append('R')
                                tree_plan.append('C')
                                plan_actions = deque(tree_plan)
                                next_act = plan_actions.popleft()

                                ##################################################
                                # print('chop tree', target_tree)
                                # print('plan_actions', plan_actions)
                                # print("execute plan", next_act) # test
                                ####################################################


                                decided = True
                    # # third: use dynamite


                    # no decide fail
                    if not decided:
                        next_act = 'L'
                        # print('fail in no items find new frontier')
    # no action
    if next_act is None:
        next_act = 'L'
        # print('fail in all judege')

    act_char = next_act.upper()
    # action logic
    if act_char == 'F':
        x, y = game_map.current_pos
        if direction_global == 'up':
            front = (x - 1, y)
        elif direction_global == 'down':
            front = (x + 1, y)
        elif direction_global == 'left':
            front = (x, y - 1)
        elif direction_global == 'right':
            front = (x, y + 1)
        cell = game_map.grid.get(front, ' ')

        if cell == 'k':
            have_key = True
        elif cell == 'a':
            have_axe = True
        elif cell == 'd':
            num_dynamites_hold += 1
        elif cell == '$':
            have_treasure = True

        if cell == '~' and have_raft and not on_raft:
            on_raft = True
            never_on_raft = False
        if on_raft and cell != '~':
            on_raft = False
            have_raft = False
    elif act_char == 'C':
        have_raft = True
    elif act_char == 'B':
        if num_dynamites_hold > 0:
            num_dynamites_hold -= 1
    action = act_char
    return act_char


    '''
    # input loop to take input from user (only returns if this is valid)
    while True:
        inp = input("Enter Action(s): ")
        inp.strip()
        final_string = ''
        for char in inp:
            if char in ['f','l','r','c','u','b','F','L','R','C','U','B']:
                final_string += char
                if final_string:
                     return final_string[0]
    '''


# helper function to print the grid
def print_grid(view):
    print('+-----+')
    for ln in view:
        print("|"+str(ln[0])+str(ln[1])+str(ln[2])+str(ln[3])+str(ln[4])+"|")
    print('+-----+')

if __name__ == "__main__":

    # checks for correct amount of arguments 
    if len(sys.argv) != 3:
        print("Usage Python3 "+sys.argv[0]+" -p port \n")
        sys.exit(1)

    port = int(sys.argv[2])

    # checking for valid port number
    if not 1025 <= port <= 65535:
        print('Incorrect port number')
        sys.exit()

    # creates TCP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
         # tries to connect to host
         # requires host is running before agent
         sock.connect(('localhost',port))
    except (ConnectionRefusedError):
         print('Connection refused, check host is running')
         sys.exit()

    # navigates through grid with input stream of data
    i=0
    j=0
    while True:
        data=sock.recv(100)
        if not data:
            exit()
        for ch in data:
            if (i==2 and j==2):
                view[i][j] = '^'
                view[i][j+1] = chr(ch)
                j+=1 
            else:
                view[i][j] = chr(ch)
            j+=1
            if j>4:
                j=0
                i=(i+1)%5
        if j==0 and i==0:


            ##############################################
            # print('------xxxxxxxxxxxxxxx--------')
            ##############################################


            print_grid(view) # COMMENT THIS OUT ON SUBMISSION
            action = get_action(view) # gets new actions
            sock.send(action.encode('utf-8'))

    sock.close()
