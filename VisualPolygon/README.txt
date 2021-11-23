Code: VisualPolygon.cpp
Author: MArco Antonio Esquivel Basaldua
Date: 11/04/2020

Code description:
Given all vertices of a polygonal free space and the location of a point robot in it,
this code gets the visual polygon assuming the robot has an omnidirectional sensor
with infinite range and the free space is polygonal and simply connected.

Inputs:
- map.txt
    File text contenant all vertices of free space.
    The first row in "map.txt" has two numbers referring to the dimensions of the map,
    the next row has one number referring the number of vertices in the free space,
    every vertex is presented by the next row indicating x, y coordinates.
- robot.txt
    Text file indicating robot location in x, y coordinates.

Outputs:
- map.png
    png image plotting free space map and robot location.
- VisualPolygon.png
    png image plotting free space map, robot location and visual polygon generated.

Compiler instructions:
- By positioning in the folder where VisualPolygon.cpp, map.txt and robot.txt are stored, use g++ compiler: 
    g++ VisualPolygon.cpp $(pkg-config --cflags --libs cairo)
- Run: 
    ./a.out
