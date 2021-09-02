#ifndef PATH_PLANNER
#define PATH_PLANNER

// System headers
#include <vector>

// CGAL headers
#include <CGAL/Cartesian.h>                                     // Kernel
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>   // Kernel
#include <CGAL/Polygon_2.h>                                     // Polygon_2
#include <CGAL/Polygon_with_holes_2.h>                          // Polygon_with_holes_2
#include <CGAL/function_objects.h>                              // Creator_uniform_2
#include <CGAL/point_generators_2.h>                            // Random_points_in_square_2
#include <CGAL/Cartesian_converter.h>                           // Convert Kernels
#include <CGAL/convex_hull_2.h>                                 // Convex hull
#include <CGAL/Convex_hull_traits_adapter_2.h>                  // for convex hull
#include <CGAL/property_map.h>                                  // for convex hull
#include<CGAL/create_straight_skeleton_2.h>                     // Straight skeleton
#include<boost/shared_ptr.hpp>                                  // for straight skeleton

// Sub-scopes
using namespace std;

// Type definitions
typedef CGAL::Exact_predicates_exact_constructions_kernel   EK;
typedef CGAL::Cartesian<double>                             IK;
typedef CGAL::Cartesian_converter<EK,IK>                    EK_to_IK;
typedef EK::Point_2                                         Point_2;
typedef vector<Point_2>                                     Point_2_container;
typedef EK::Segment_2                                       Segment_2;
typedef CGAL::Polygon_2<EK>                                 Polygon_2;
typedef CGAL::Polygon_with_holes_2<EK>                      Polygon_with_holes_2;
typedef vector<Polygon_2>                                   Pgn_2_container;
typedef vector<Polygon_with_holes_2>                        Pgn_with_holes_2_container;
typedef CGAL::Creator_uniform_2<int, Point_2>               Creator;
typedef CGAL::Random_points_in_square_2<Point_2, Creator>   Point_generator;
typedef CGAL::Convex_hull_traits_adapter_2<EK, CGAL::Pointer_property_map<Point_2>::type > Convex_hull_traits_2;
typedef CGAL::Exact_predicates_inexact_constructions_kernel EIK;
typedef CGAL::Straight_skeleton_2<EIK>                      Ss;
typedef boost::shared_ptr<Ss>                               SsPtr;
typedef CGAL::Cartesian_converter<EIK,IK>                   EIK_to_IK;

// Constant variables
#define MIN_POLY_VERTEX 5
#define MAX_POLY_VERTEX 8
#define MIN_POLY_RADIUS 18.0
#define MAX_POLY_RADIUS 19.0
#define SAFE_ZONE_BUFFER 6
#define MIN_CONTAINER_VERTEX 100
#define MAX_CONTAINER_VERTEX 110
#define MIN_CONTAINER_RADIUS 250.0
#define MAX_CONTAINER_RADIUS 251.0
#define SHAPE_OFFSET 1.01
#define MICRO_INSET 2
#define INSET 3
#define PI 3.14159265
#define CIRC_SIDES 30

// Function declarations
Polygon_2 get_random_polygon(int min_vert, int max_vert, double min_rad, double max_rad);
Polygon_2 get_random_container();

#endif