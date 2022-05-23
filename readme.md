# A*: Single Robot Path Planning Algorithm - MATLAB
Single Robot Path Planning.

coding based on nodes.

In semi-unknown environment:
 - known initial map.
 - detect new obstacles (remapping is needed). 

note: with dir (Direction) as a parameter in selecting next node.

## run code

RUN_Astar.m: A* path planning

RUN_Astar_Remapping: A* pp for map with changing edge costs


## myAStar function
two mode:  8 and 4 neighbor nodes

neighbors4: arrange neighbors based on robot direction

neighbors8

![obstacle9](https://user-images.githubusercontent.com/32360441/169779558-273d177a-4dc6-4cc6-b59b-afaf988aa8e6.jpg)
![obstacles19](https://user-images.githubusercontent.com/32360441/169779563-dfb80fea-d312-406a-aa97-3ee6763243d1.jpg)
![srpp-astar2-1](https://user-images.githubusercontent.com/32360441/169691636-c0cc37b4-1818-44d0-b457-5d8df444e137.jpg)
