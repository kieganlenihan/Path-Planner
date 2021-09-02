#ifndef MOVER
#define MOVER

// OpenCV headers
#include <opencv2/opencv.hpp>           // Mat, circle, imshow, waitKey

// CGAL headers
#include <CGAL/Aff_transformation_2.h>  // transformations
#include <CGAL/Vector_2.h>              // vector

// Working directory headers
#include "path_planner.h"                   // type definitions and CGAL Kernels

// Sub-scopes
using namespace cv;

// Type definitions
typedef CGAL::Aff_transformation_2<EK>      Transformation;
typedef CGAL::Vector_2<EK>                  Vector;
typedef CGAL::Bbox_2                        Bbox_2;
typedef CGAL::Translation                   TRANSLATION;
typedef CGAL::Scaling                       SCALING;

class Mover {
    private:
        Polygon_2 container, robot, box;
        Point_2 start, finish;
    public:
        EK_to_IK to_inexact;
        EIK_to_IK to_inexact_EIK;
        void test();
        void set_input_geometry(Polygon_2 rob, Polygon_2 cont);
        Point_2 get_pt_in_part_of_shape(double radius, Transformation transform, Polygon_2 shape);
        void set_start_finish_pts();
        Polygon_2 transform_shape(Polygon_2 shape, Transformation transform);
        Polygon_with_holes_2 transform_shape(Polygon_with_holes_2 shape, Transformation transform);
        Point_2 transform_point(Point_2 pt, Transformation transform);
        Polygon_2 snap_origin_to_bl(Polygon_2 shape);
        Polygon_2 snap_origin_to_tr(Polygon_2 shape);
        Polygon_2 circumscribed_polygon(int n_sides, double radius, double c_x, double c_y);
        Polygon_2 min_rect(Polygon_2 shape);
        Polygon_2 to_convex_poly(Polygon_2 shape);
        Polygon_2 min_circle_profile(Polygon_2 shape);
        Polygon_2 get_largest_hole(Polygon_with_holes_2 shape);
        SsPtr construct_straight_skeleton(Polygon_2 shape);
        // Test functions
        void feasible_set_example();
        void starting_condition();
        void circle_path();
};

 #endif