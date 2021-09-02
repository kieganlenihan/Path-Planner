// System headers
#include <chrono>                               // timing 
#include <iostream>                             // cout, command line args
#include <string>                               // string
#include <assert.h>                             // assert

// OpenCV headers
#include <opencv2/opencv.hpp>                   // Mat, imshow, waitKey

// CGAL headers
#include <CGAL/random_polygon_2.h>              // random polygon
// #include <CGAL/draw_polygon_2.h>                // polygon visualization
// #include <CGAL/draw_polygon_with_holes_2.h>     // polygon with holes visualization
#include <CGAL/minkowski_sum_2.h>               // minkowski sum

// Working directory headers
#include "headers/path_planner.h"               // type definitions and CGAL Kernels
#include "headers/print.h"                      // pretty print polygons, straight skeletons
#include "headers/mover.h"                      // path calcualtions
#include "headers/visualize.h"                  // visualize

// Sub-scopes
using namespace std;
using namespace std::chrono;
using namespace cv;

Polygon_2 get_random_polygon(int min_vert, int max_vert, double min_rad, double max_rad) {
    Polygon_2 polygon, outer;
    CGAL::Random rand;
    Point_2_container point_set;
    int n_vertices = rand.get_int(min_vert, max_vert);
    double radius = rand.get_double(min_rad, max_rad);
    CGAL::copy_n_unique(Point_generator(radius), n_vertices, back_inserter(point_set));
    CGAL::random_polygon_2(point_set.size(), back_inserter(polygon), point_set.begin());
    Polygon_2 offseter;
    offseter.push_back(Point_2(-MICRO_INSET, -MICRO_INSET));
    offseter.push_back(Point_2(MICRO_INSET, -MICRO_INSET));
    offseter.push_back(Point_2(MICRO_INSET, MICRO_INSET));
    offseter.push_back(Point_2(-MICRO_INSET, MICRO_INSET));
    Polygon_with_holes_2 sum = CGAL::minkowski_sum_2(polygon, offseter);
    outer = sum.outer_boundary();
    if (polygon.is_clockwise_oriented()) {
        polygon.reverse_orientation();
    }
    return polygon;
}

Polygon_2 get_random_container() {
    Polygon_2 polygon;
    polygon = get_random_polygon(MIN_CONTAINER_VERTEX, MAX_CONTAINER_VERTEX, MIN_CONTAINER_RADIUS, MAX_CONTAINER_RADIUS);
    Polygon_2 poly, outer;
    Polygon_2 offseter;
    offseter.push_back(Point_2(-INSET, -INSET));
    offseter.push_back(Point_2(INSET, -INSET));
    offseter.push_back(Point_2(INSET, INSET));
    offseter.push_back(Point_2(-INSET, INSET));
    Polygon_with_holes_2 sum = CGAL::minkowski_sum_2(polygon, offseter);
    outer = sum.outer_boundary();
    return outer;
}

int main() {
    if (__cplusplus == 201703L) std::cout << "C++17\n";
    else if (__cplusplus == 201402L) std::cout << "C++14\n";
    else if (__cplusplus == 201103L) std::cout << "C++11\n";
    else if (__cplusplus == 199711L) std::cout << "C++98\n";
    else std::cout << "pre-standard C++\n";
    cout << "\n-> Welcome to Path Planner!\n\nType 'generate' for to watch an animation of 2D robot path planning!\n\n";
    string input_str;
    // cin >> input_str;
    input_str = "generate";
    if (input_str == "generate") {
        CGAL::Random rand;
        chrono::time_point<chrono::steady_clock> start = chrono::steady_clock::now();
        Polygon_2 robot = get_random_polygon(MIN_POLY_VERTEX, MAX_POLY_VERTEX, MIN_POLY_RADIUS - MICRO_INSET, MAX_POLY_RADIUS - MICRO_INSET);
        Polygon_2 container = get_random_container();
        Mover algo;
        algo.set_input_geometry(robot, container);
        algo.starting_condition();
        algo.circle_path();
    }
    return EXIT_SUCCESS;
}