/*
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
*/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <map>
#include <cairo.h>
#include <limits>
#include <algorithm>
//#include <gtk/gtk.h>
using namespace std;

#define Pi 3.14159265359

void readMap(pair<double,double>&, vector<pair<double,double> >&, pair<double,double>&);
pair<int,pair<double,double> > solve2x2(pair<double,double>,pair<double,double>,pair<double,double>,pair<double,double>);
void visual(pair<double,double>, vector<pair<double,double> >, vector<pair<double,double> >, vector<pair<double,double> >&);
double dist(pair<double,double>, pair<double,double>);
void sensorPosition(pair<double,double>, vector<pair<double,double> >&);
void GraphCairo_polygon(pair<double,double>, vector<pair<double,double> >, vector<pair<double,double> >, pair<double,double>);
void GraphCairo_map(pair<double,double>, vector<pair<double,double> >, pair<double,double>);




int main(void){

    pair<double,double> dimension;                    // map dimensions
    pair<double,double> robot;                       // robot location
    vector<pair<double,double> > mapCoordinates;    // each vertice coordinates of free space
    //vector<pair<double,pair<double,double> > > mapCoordinates_byAngle;    // each vertice ordered by angle respect of robot location
    vector<pair<double,double> > polygon;          // vertices on visualization polygon
    vector<pair<double,double> > sensor;          // virtual omnidirectional sensor arround the robot

    // File with map specifications is read
    readMap(dimension, mapCoordinates, robot);

    sensorPosition(robot, sensor);

    // Visual polygon is generated
    visual(robot, mapCoordinates, sensor, polygon);

    
    // map.png and VisualPolygon.png are plotted
    GraphCairo_map(dimension, mapCoordinates, robot);
    GraphCairo_polygon(dimension, mapCoordinates, polygon, robot);

    return 0;
}


void readMap(pair<double,double>& dimension,vector<pair<double,double> >& mapCoordinates,pair<double,double>& robot){
    int n;
    string line;
    ifstream input;

    input.open("robot.txt");
    if(input.is_open()){
        input >> robot.first;
        input >> robot.second;
    }
    input.close();

    input.open("map.txt");
    if(input.is_open()){
        input >> dimension.first;
        input >> dimension.second;

        input >> n;

        pair<double,double> xy;
        for(int i=0; i<n; i++){
            input >> xy.first;
            input >> xy.second;

            mapCoordinates.push_back(xy);
        }
    }
    input.close();
}


void visual(pair<double,double> robot, vector<pair<double,double> > mapCoordinates, vector<pair<double,double> > sensor, vector<pair<double,double> >& polygon){
    pair <double, double> p2, p3, p4, p_polygon, xy;
    pair <int, pair<double,double> > intersection;

    for(int i=0; i<sensor.size(); i++){
        p2 = sensor[i];

        double last_distance = HUGE_VAL;
        double new_distance;

        
        // Check intersections for every edge
        for(int j=0; j<mapCoordinates.size(); j++){
            
            if(j == mapCoordinates.size()-1){
                p3 = mapCoordinates[j];
                p4 = mapCoordinates[0];
            }
            else{
                p3 = mapCoordinates[j];
                p4 = mapCoordinates[j+1];
            }

            // find intersection
            intersection = solve2x2(robot, p2, p3, p4);
            int state = intersection.first;
            xy = intersection.second;
            double angle = atan2(xy.second-robot.second, xy.first-robot.first);
            double angle2 = atan2(sensor[i].second - robot.second, sensor[i].first - robot.first);

            if(state == 1){
                int cuadrant = 0;
                if(angle2 <= -Pi/2.0 and angle <= -Pi/2.0) cuadrant = 1;
                if(angle2 > -Pi/2.0 and angle2 <= 0.0 and angle > -Pi/2.0 and angle <= 0.0) cuadrant = 1;
                if(angle2 > 0.0 and angle2 <= Pi/2.0 and angle > 0.0 and angle <= Pi/2.0) cuadrant = 1;
                if(angle2 > Pi/2.0 and angle > Pi/2.0) cuadrant = 1;

                if(cuadrant){
                    double min_x = fmin(p3.first, p4.first);
                    double max_x = fmax(p3.first, p4.first);
                    double min_y = fmin(p3.second, p4.second);
                    double max_y = fmax(p3.second, p4.second);

                    if(xy.first >= min_x and xy.first <= max_x and xy.second >= min_y and xy.second <= max_y){
                        new_distance = dist(xy, robot);
                        if(new_distance < last_distance){
                            p_polygon = xy;
                            last_distance = new_distance;
                        }
                    }
                }
            }
        }
        polygon.push_back(p_polygon);

    }
}

pair<int,pair<double,double> > solve2x2(pair<double,double> p1,pair<double,double> p2,pair<double,double> p3,pair<double,double> p4){
    pair<double,double> sol(0.0,0.0);
    int state;
    double epsilon = 1e-3;

    double a = p1.second - p2.second;
    double b = p2.first - p1.first;
    double c = p3.second - p4.second;
    double d = p4.first - p3.first;
    double s1 = p2.first*p1.second - p1.first*p2.second;
    double s2 = p4.first*p3.second - p3.first*p4.second;

    double det = a*d - b*c;

    if(fabs(det) > epsilon){
        state = 1;
        sol.first = (s1*d - b*s2)/det;
        sol.second = (a*s2 - s1*c)/det;
    }
    else if(fabs(s1*(c/a)-s2) < epsilon)
        state = -1;
    else
        state = 0;

    return make_pair(state,sol);
}

void sensorPosition(pair<double,double> robot, vector<pair<double,double> >& sensor){

    double delta = Pi/360.0;
    double angle = -Pi;

    double x, y;
    for(int i=0; i<720; i++){
        x = robot.first + cos(angle);
        y = robot.second + sin(angle);

        sensor.push_back(make_pair(x,y));
        angle += delta;
    }
}


double dist(pair<double,double> a, pair<double,double> b){
    return sqrt((b.first-a.first)*(b.first-a.first) + (b.second-a.second)*(b.second-a.second));
}


void GraphCairo_polygon(pair<double,double> dimension, vector<pair<double,double> > mapCoordinates,vector<pair<double,double> > polygon, pair<double,double> robot){
    int scale = 10;
    if(dimension.first <= 100 or dimension.second <= 100){
        dimension.first *= scale;
        dimension.second *= scale;
        for(int i=0; i<mapCoordinates.size(); i++){
            mapCoordinates[i].first *= scale;
            mapCoordinates[i].second *= scale;
        }
        for(int i=0; i<polygon.size(); i++){
            polygon[i].first *= scale;
            polygon[i].second *= scale;
        }
        robot.first *= scale;
        robot.second *= scale;
    }
    else
        scale = 1;


    cairo_surface_t *surface;
    cairo_t *cr,*line;
    surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32,dimension.first,dimension.second);
    cr = cairo_create (surface);
    line = cairo_create (surface);
    cairo_rectangle (cr,0,0,dimension.first,dimension.second);
    cairo_set_source_rgb (cr,0,0,0);
    cairo_fill(cr);
    //////

    cairo_set_source_rgb (cr,1,1,1);
    cairo_move_to (cr,mapCoordinates[0].first,dimension.second-mapCoordinates[0].second);
    for(int i=1; i<mapCoordinates.size();i++){
        cairo_line_to (cr,mapCoordinates[i].first,dimension.second-mapCoordinates[i].second);
    }

    cairo_close_path(cr);
    cairo_stroke_preserve(cr);
    cairo_fill(cr);

    cairo_set_source_rgb (cr,0,0,1);
    cairo_move_to (cr,polygon[0].first,dimension.second-polygon[0].second);
    for(int i=1; i<polygon.size();i++){
        cairo_line_to (cr,polygon[i].first,dimension.second-polygon[i].second);
    }

    cairo_close_path(cr);
    cairo_stroke_preserve(cr);
    cairo_fill(cr);


    cairo_set_source_rgb (cr,1,0,0);
    cairo_arc(cr,robot.first,dimension.second-robot.second,dimension.first/30.0,0,2*Pi);
    cairo_fill(cr);


    //////
    cairo_surface_write_to_png (surface,"VisualPolygon.png");
    cairo_destroy (cr);
    cairo_destroy (line);
    cairo_surface_destroy (surface);
}



void GraphCairo_map(pair<double,double> dimension, vector<pair<double,double> > mapCoordinates, pair<double,double> robot){
    int scale = 10;
    if(dimension.first <= 100 or dimension.second <= 100){
        dimension.first *= scale;
        dimension.second *= scale;
        for(int i=0; i<mapCoordinates.size(); i++){
            mapCoordinates[i].first *= scale;
            mapCoordinates[i].second *= scale;
        }
        
        robot.first *= scale;
        robot.second *= scale;
    }
    else
        scale = 1;


    cairo_surface_t *surface;
    cairo_t *cr,*line;
    surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32,dimension.first,dimension.second);
    cr = cairo_create (surface);
    line = cairo_create (surface);
    cairo_rectangle (cr,0,0,dimension.first,dimension.second);
    cairo_set_source_rgb (cr,0,0,0);
    cairo_fill(cr);
    //////

    cairo_set_source_rgb (cr,1,1,1);
    cairo_move_to (cr,mapCoordinates[0].first,dimension.second-mapCoordinates[0].second);
    for(int i=1; i<mapCoordinates.size();i++){
        cairo_line_to (cr,mapCoordinates[i].first,dimension.second-mapCoordinates[i].second);
    }

    cairo_close_path(cr);
    cairo_stroke_preserve(cr);
    cairo_fill(cr);

    cairo_set_source_rgb (cr,1,0,0);
    cairo_arc(cr,robot.first,dimension.second-robot.second,dimension.first/30.0,0,2*Pi);
    cairo_fill(cr);

    //////
    cairo_surface_write_to_png (surface,"map.png");
    cairo_destroy (cr);
    cairo_destroy (line);
    cairo_surface_destroy (surface);
}