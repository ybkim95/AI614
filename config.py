from random import uniform

N_ITER = 300
STEP_SIZE = 0.8
RADIUS = 1
# OBSTACLES = [
#     # (x,y,r)
#     # (5,5, uniform(0.5,1.5)), 
#     (9,3, uniform(0.5,1.5)), 
#     (1.5,9, uniform(0.5,1.5)), 
#     (1,4, uniform(0.5,1.5)), 
#     # (6,10, uniform(0.5,1.5)), 
#     (6,0, uniform(0.5,1.5))
# ]

CONSISTENT_OBSTACLES = [
    (3.5,6, uniform(1.75,1.8)), 
    (6.2,3.6, uniform(1.2,1.3)), 
    (1.5,9, uniform(0.5,1.5)), 
    (1,4, uniform(0.5,1.5)), 
    (6,10, uniform(0.5,1.5)), 
    (5,0.2, uniform(0.5,1.5))
]

RRT_OBSTACLES = [
    (4.5,4.5, 2.1) 
    # (1.5,9, 0.7), 
    # (1,4, 0.5), 
    # (5,0.2, 1.0)
]

START = (1,1)
END = (7,7)
CONFIG = (START, END, RRT_OBSTACLES)

GAMMA = 3