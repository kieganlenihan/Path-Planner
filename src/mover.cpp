// System headers
#include <math.h>                           // sin, cos, sqrt

// CGAL headers
#include <CGAL/minkowski_sum_2.h>           // minkowski sum
#include <CGAL/enum.h>                      // oriented side
#include <CGAL/random_polygon_2.h>          // random polygon
#include <CGAL/min_quadrilateral_2.h>       // OMBB
#include <CGAL/Segment_2.h>                 // line length

// Working directory headers
#include "headers/path_planner.h"           // type definitions and CGAL Kernels
#include "headers/print.h"                  // pretty print polygons
#include "headers/mover.h"                  // Path class declarations
#include "headers/visualize.h"              // Visualizations class
#include "headers/optimize.h"               // Genetic algorithm for path planning

// Sub-scopes
using namespace std;

void Mover::set_input_geometry(Polygon_2 rob, Polygon_2 cont) {
    Transformation stretch(0.5, 0, 0, 0, 2.0, 0, 1);
    robot = transform_shape(rob, stretch);
    container = snap_origin_to_bl(cont);
    container.reverse_orientation();
    set_start_finish_pts();
}

void Mover::set_start_finish_pts() {
    Polygon_2::Vertex_iterator xmin, xmax, ymin, ymax;
    Point_2 l, r, b, t;
    xmin = robot.left_vertex();
    xmax = robot.right_vertex();
    ymin = robot.bottom_vertex();
    ymax = robot.top_vertex();
    l = *xmin;
    r = *xmax;
    b = *ymin;
    t = *ymax;
    Polygon_2 bbox;
    bbox.push_back(Point_2(l.x() - SAFE_ZONE_BUFFER, b.y() - SAFE_ZONE_BUFFER));
    bbox.push_back(Point_2(r.x() + SAFE_ZONE_BUFFER, b.y() - SAFE_ZONE_BUFFER));
    bbox.push_back(Point_2(r.x() + SAFE_ZONE_BUFFER, t.y() + SAFE_ZONE_BUFFER));
    bbox.push_back(Point_2(l.x() - SAFE_ZONE_BUFFER, t.y() + SAFE_ZONE_BUFFER));
    IK::Point_2 p1, p2, p3, p4;
    p1 = to_inexact(l);
    p2 = to_inexact(r);
    p3 = to_inexact(b);
    p4 = to_inexact(t);
    double x_shift = (p2.x() - p1.x()) / 2 - p2.x();
    double y_shift = (p4.y() - p3.y()) / 2 - p4.y();
    Transformation shift(1, 0, x_shift, 0, 1, y_shift, 1);
    box = transform_shape(bbox, shift);
    robot = transform_shape(robot, shift);
    Polygon_2 outer_poly, hole;
    Polygon_with_holes_2 holed_cont(outer_poly);
    holed_cont.add_hole(container);
    Polygon_with_holes_2 sum = CGAL::minkowski_sum_2(holed_cont, box);
    Polygon_2::Vertex_iterator xl, xr;
    Point_2 s, f;
    Point_2 s_(MAX_CONTAINER_RADIUS * 2, 0);
    Point_2 f_(-MAX_CONTAINER_RADIUS * 2, 0);
    for (Polygon_with_holes_2::Hole_iterator hi = sum.holes_begin(); hi < sum.holes_end(); ++hi) {
        hole = *hi;
        xl = hole.left_vertex();
        xr = hole.right_vertex();
        s = *xl;
        f = *xr;
        if (s.x() < s_.x()) {
            s_ = s;
        }
        if (f.x() > f_.x()) {
            f_ = f;
        }
    }
    start = s_;
    finish = f_;
}

Polygon_2 Mover::transform_shape(Polygon_2 shape, Transformation transform) {
    Polygon_2 transformed_shape;
    Point_2 p;
    for (Polygon_2::Vertex_iterator vi = shape.vertices_begin(); vi < shape.vertices_end(); ++vi) {
        p = *vi;
        transformed_shape.push_back(transform(p));
    }
    return transformed_shape;
}

// Overloaded transform_shape for Polygon_with_holes_2
Polygon_with_holes_2 Mover::transform_shape(Polygon_with_holes_2 shape, Transformation transform) {
    Polygon_with_holes_2 transformed_shape(transform_shape(shape.outer_boundary(), transform));
    Polygon_2 hole, transformed_hole;
    for (Polygon_with_holes_2::Hole_iterator hi = shape.holes_begin(); hi < shape.holes_end(); ++hi) {
        hole = *hi;
        transformed_hole = transform_shape(hole, transform);
        transformed_shape.add_hole(transformed_hole);
    }
    return transformed_shape;
}

Point_2 Mover::transform_point(Point_2 pt, Transformation transform) {
    return transform(pt);
}

Polygon_2 Mover::snap_origin_to_bl(Polygon_2 shape) {
    Polygon_2::Vertex_iterator xmin = shape.left_vertex();
    Polygon_2::Vertex_iterator ymin = shape.bottom_vertex();
    Point_2 p1, p2;
    p1 = *xmin;
    p2 = *ymin;
    Transformation snap(1, 0, -p1.x(), 0, 1, -p2.y(), 1);
    return transform_shape(shape, snap);
}

Polygon_2 Mover::snap_origin_to_tr(Polygon_2 shape) {
    Polygon_2::Vertex_iterator xmax = shape.right_vertex();
    Polygon_2::Vertex_iterator ymax = shape.top_vertex();
    Point_2 p1, p2;
    p1 = *xmax;
    p2 = *ymax;
    Transformation snap(1, 0, -p1.x(), 0, 1, -p2.y(), 1);
    return transform_shape(shape, snap);
}

Polygon_2 Mover::circumscribed_polygon(int n_sides, double radius, double c_x, double c_y) {
    double theta = 2 * PI / n_sides;
    double h = radius / cos(theta / 2);
    Polygon_2 shape;
    double phi;
    for (int i = 0; i < n_sides; ++i) {
        phi = 2 * PI * i / n_sides;
        shape.push_back(Point_2(radius * cos (phi) + c_x, radius * sin (phi) + c_y));
    }
    return shape;
}

Polygon_2 Mover::to_convex_poly(Polygon_2 shape) {
    vector<Point_2> points;
    Point_2 pt;
    for (Polygon_2::Vertex_iterator vi = shape.vertices_begin(); vi < shape.vertices_end(); ++vi) {
        pt = *vi;
        points.push_back(pt);
    }
    vector<size_t> indices(points.size()), out;
    iota(indices.begin(), indices.end(), 0);
    CGAL::convex_hull_2(indices.begin(), indices.end(), back_inserter(out), Convex_hull_traits_2(CGAL::make_property_map(points)));
    Polygon_2 convex_poly;
    for (size_t i : out) {
        convex_poly.push_back(points[i]);
    }
    return convex_poly;
}

Polygon_2 Mover::min_rect(Polygon_2 shape) {
    Polygon_2 OMBB;
    Polygon_2 poly_convex = to_convex_poly(shape);
    CGAL::min_rectangle_2(poly_convex.vertices_begin(), poly_convex.vertices_end(), back_inserter(OMBB));
    return OMBB;
}

Polygon_2 Mover::min_circle_profile(Polygon_2 shape) {
    Polygon_2 OMBB = min_rect(shape);
    Segment_2 line;
    IK::Segment_2 line_IK, line1, line2;
    double length_ = MAX_POLY_RADIUS * 100;
    double length;
    int i = 0;
    vector<IK::Segment_2> lines;
    for (Polygon_2::Edge_const_iterator it = OMBB.edges_begin(); it < OMBB.edges_end(); ++it) {
        line = *it;
        line_IK = to_inexact(line);
        length = sqrt(line_IK.squared_length());
        if (length < length_) {
            length_ = length;
        }
        if (i < 2) {
            lines.push_back(line_IK);
        }
    }
    line1 = lines.at(0);
    line2 = lines.at(1);
    IK::Point_2 p1 = line1.source();
    IK::Point_2 p2 = line2.target();
    double c_x = (p1.x() + p2.x()) / 2;
    double c_y = (p1.y() + p2.y()) / 2;
    double radius = length_ / 2;
    Polygon_2 circ = circumscribed_polygon(CIRC_SIDES, radius, c_x, c_y);
    return circ;
}

Polygon_2 Mover::get_largest_hole(Polygon_with_holes_2 shape) {
    CGAL::Lazy_exact_nt<__gmp_expr<mpq_t, mpq_t> > area_ = 0;
    CGAL::Lazy_exact_nt<__gmp_expr<mpq_t, mpq_t> > area;
    Polygon_2 hole, largest;
    for (Polygon_with_holes_2::Hole_iterator hi = shape.holes_begin();  hi < shape.holes_end(); ++hi) {
        hole = *hi;
        if (hole.is_clockwise_oriented()) {
            hole.reverse_orientation();
        }
        area = hole.area();
        if (area > area_) {
            area_ = area;
            largest = hole;
        }
    }
    return largest;
}

SsPtr Mover::construct_straight_skeleton(Polygon_2 shape) {
    SsPtr iss = CGAL::create_interior_straight_skeleton_2(shape.vertices_begin(), shape.vertices_end());
    return iss;
}

// ---------- Test Functions ---------- //

void Mover::feasible_set_example() {
    Polygon_2 outer_poly;
    Polygon_with_holes_2 holed_cont(outer_poly);
    holed_cont.add_hole(container);
    Polygon_with_holes_2 sum;
    // Set visualizations
    Visualizations vis;
    vis.set_feasible_set_img(container);
    // Rotate shape and calculate minkowski sum
    Transformation rotate_180(cos(PI), -sin(PI), 0, sin(PI), cos(PI), 0, 1);
    Polygon_2 flipped_shape, rotated_shape;
    for (double theta = 0; theta < 2 * PI; theta += 2 * PI / n_frames) {
        Transformation rotate(cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 1);
        rotated_shape = transform_shape(robot, rotate);
        flipped_shape = transform_shape(rotated_shape, rotate_180);
        sum = CGAL::minkowski_sum_2(holed_cont, flipped_shape);
        vis.draw_feasible_set(container, sum, rotated_shape);
    }
    vis.outputVideo.release();
    destroyAllWindows();
}

void Mover::starting_condition() {
    Population pop;
    // pop.set_genetic_algo_parameters
    Visualizations vis;
    vis.set_feasible_set_img(container);
    vis.draw_starting_point(container, robot, start, finish, box);
    vis.visualize_img(0);
}

void Mover::circle_path() {
    Polygon_2 OMBB = min_rect(robot);
    Polygon_2 circ = min_circle_profile(robot);
    Polygon_2 outer_poly;
    Polygon_with_holes_2 holed_cont(outer_poly);
    holed_cont.add_hole(container);
    Polygon_with_holes_2 sum, sum_;
    sum = CGAL::minkowski_sum_2(holed_cont, circ);
    Polygon_2 sum_boundary = get_largest_hole(sum);

    // Polygon_2 square = min_rect(circ);
    // sum_ = CGAL::minkowski_sum_2(holed_cont, square);
    // Polygon_2 sum_boundary_ = get_largest_hole(sum_);
    SsPtr iss = construct_straight_skeleton(sum_boundary);

    Visualizations vis;
    vis.set_feasible_set_img(container);
    vis.draw_minimal_path(container, robot, start, finish, circ, OMBB, sum_boundary, iss);
    vis.visualize_img(0);
}