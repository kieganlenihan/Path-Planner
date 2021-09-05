// System headers
#include <algorithm>                        // max
#include <math.h>                           // sin, cos

// OpenCV headers
#include <opencv2/opencv.hpp>               // Mat
#include <opencv2/core/types.hpp>           // Point2d
#include <opencv2/imgproc.hpp>              // fillPoly, line
#include <opencv2/videoio.hpp>              // Video write

// Working directory headers
#include "headers/path_planner.h"           // type definitions and CGAL Kernels
#include "headers/print.h"                  // pretty print polygons
#include "headers/mover.h"                  // path class definitions
#include "headers/colors.h"                 // colors
#include "headers/visualize.h"              // visualization

// Sub-scopes
using namespace std;
using namespace cv;

void Visualizations::set_img_params(CGAL::Polygon_2<IK> shape) {
    CGAL::Polygon_2<IK>::Vertex_iterator xmin = shape.left_vertex();
    CGAL::Polygon_2<IK>::Vertex_iterator ymin = shape.bottom_vertex();
    CGAL::Polygon_2<IK>::Vertex_iterator xmax = shape.right_vertex();
    CGAL::Polygon_2<IK>::Vertex_iterator ymax = shape.top_vertex();
    IK::Point_2 p1, p2, p3, p4;
    p1 = *xmin;
    p2 = *ymin;
    p3 = *xmax;
    p4 = *ymax;
    w = p3.x() - p1.x();
    h = p4.y() - p2.y();
    SCALEFACTOR = MAX_IMG_DIM / max(w, h);
}

void Visualizations::set_output_image(int x, int y) {
    img = Mat(y, x, CV_8UC3, Scalar(200, 200, 200));
}

// Overloaded transform_shape for CGAL::Polygon_2<IK>
CGAL::Polygon_2<IK> Visualizations::transform_shape_IK(CGAL::Polygon_2<IK> shape, CGAL::Aff_transformation_2<IK> transform) {
    CGAL::Polygon_2<IK> transformed_shape;
    IK::Point_2 p;
    for (CGAL::Polygon_2<IK>::Vertex_iterator vi = shape.vertices_begin(); vi < shape.vertices_end(); ++vi) {
        p = *vi;
        transformed_shape.push_back(transform(p));
    }
    return transformed_shape;
}

// Overloaded transform_shape for CGAL::Polygon_with_holes_2<IK>
CGAL::Polygon_with_holes_2<IK> Visualizations::transform_shape_IK(CGAL::Polygon_with_holes_2<IK> shape, CGAL::Aff_transformation_2<IK> transform) {
    CGAL::Polygon_with_holes_2<IK> transformed_shape(transform_shape_IK(shape.outer_boundary(), transform));
    CGAL::Polygon_2<IK> hole, transformed_hole;
    for (CGAL::Polygon_with_holes_2<IK>::Hole_iterator hi = shape.holes_begin(); hi < shape.holes_end(); ++hi) {
        hole = *hi;
        transformed_hole = transform_shape_IK(hole, transform);
        transformed_shape.add_hole(transformed_hole);
    }
    return transformed_shape;
}

IK::Point_2 Visualizations::transform_point_IK(IK::Point_2 pt, CGAL::Aff_transformation_2<IK> transform) {
    return transform(pt);
}

void Visualizations::draw_shape(CGAL::Polygon_2<IK> shape, Scalar line_color = COLORS_empty, Scalar fill_color = COLORS_empty) {
    if (line_color != COLORS_empty) {
        IK::Segment_2 edge;
        IK::Point_2 p1, p2;
        for (CGAL::Polygon_2<IK>::Edge_const_iterator it = shape.edges_begin(); it < shape.edges_end(); ++it) {
            edge = *it;
            p1 = edge.source();
            p2 = edge.target();
            line(img, Point(p1.x(), p1.y()), Point(p2.x(), p2.y()), line_color, LINEWIDTH);
        }
    }
    if (fill_color != COLORS_empty) {
        IK::Point_2 p;
        int vtx = shape.size();
        if (vtx != 0) {
            Point poly_points[1][vtx];
            int i = 0;
            for (CGAL::Polygon_2<IK>::Vertex_iterator vi = shape.vertices_begin(); vi < shape.vertices_end(); ++vi) {
                p = *vi;
                poly_points[0][i] = Point(p.x(), p.y());
                i++;
            }
            const Point* ppt[1] = {poly_points[0]};
            int npt[] = {vtx};
            int linetype = LINE_8;
            fillPoly(img, ppt, npt, 1, fill_color, linetype);
        }
    }
    
}

// Overloaded draw_shape for Polygon_with_holes_2
void Visualizations::draw_shape(CGAL::Polygon_with_holes_2<IK> shape, Scalar line_color, Scalar fill_color) {
    draw_shape(shape.outer_boundary(), line_color, fill_color);
    CGAL::Polygon_2<IK> hole;
    for (CGAL::Polygon_with_holes_2<IK>::Hole_iterator hi = shape.holes_begin(); hi < shape.holes_end(); ++hi) {
        hole = *hi;
        draw_shape(hole, line_color, fill_color);
    }
}

void Visualizations::draw_point(IK::Point_2 pt, Scalar color) {
    circle(img, Point(pt.x(), pt.y()), POINT_RADIUS, color, FILLED, LINE_8);
}

CGAL::Aff_transformation_2<IK> Visualizations::align_shape(CGAL::Polygon_2<IK> shape, double x, double y, double shift) {
    CGAL::Polygon_2<IK>::Vertex_iterator xmin = shape.left_vertex();
    CGAL::Polygon_2<IK>::Vertex_iterator ymin = shape.bottom_vertex();
    IK::Point_2 p1, p2;
    p1 = *xmin;
    p2 = *ymin;
    double dx = p1.x() - x;
    double dy = p2.y() - y;
    CGAL::Aff_transformation_2<IK> transform(SCALEFACTOR, 0, -dx * SCALEFACTOR + shift, 0, SCALEFACTOR, -dy * SCALEFACTOR + shift, 1);
    return transform;
}

CGAL::Aff_transformation_2<IK> Visualizations::align_shape(CGAL::Polygon_with_holes_2<IK> shape, double x, double y, double shift) {
    return align_shape(shape.outer_boundary(), x, y, shift);
}

CGAL::Polygon_2<IK> Visualizations::convert_polygon(Polygon_2 shape) {
    CGAL::Polygon_2<IK> shape_IK;
    Point_2 pt;
    IK::Point_2 pt_IK;
    for (Polygon_2::Vertex_iterator vi = shape.vertices_begin(); vi < shape.vertices_end(); ++vi) {
        pt = *vi;
        pt_IK = to_inexact(pt);
        shape_IK.push_back(pt_IK);
    }
    return shape_IK;
}

CGAL::Polygon_with_holes_2<IK> Visualizations::convert_polygon(Polygon_with_holes_2 shape) {
    CGAL::Polygon_with_holes_2<IK> shape_IK(convert_polygon(shape.outer_boundary()));
    Polygon_2 hole;
    CGAL::Polygon_2<IK> hole_IK;
    for (Polygon_with_holes_2::Hole_iterator hi = shape.holes_begin(); hi < shape.holes_end(); ++hi) {
        hole = *hi;
        hole_IK = convert_polygon(hole);
        shape_IK.add_hole(hole_IK);
    }
    return shape_IK;
}

void Visualizations::draw_straight_skeleton(Ss const& iss, Scalar line_color, CGAL::Aff_transformation_2<IK> transform) {
    EIK::Point_2 pt, pt_;
    IK::Point_2 pt1, pt2;
    IK::Segment_2 edge;
    Scalar color;
    int i = 0;
    for (Ss::Halfedge_const_iterator it = iss.halfedges_begin(); it != iss.halfedges_end(); ++it) {
        pt_ = it->opposite()->vertex()->point();
        pt = it->vertex()->point();
        pt1 = transform(to_inexact_EIK(pt_));
        pt2 = transform(to_inexact_EIK(pt));
        if (i % 2 == 0) {
            color = COLORS_blue;
        }
        else {
            color = COLORS_blue;
        }
        line(img, Point(pt1.x(), pt1.y()), Point(pt2.x(), pt2.y()), color, LINEWIDTH);
        i++;
    }
}

void Visualizations::visualize_img(int wait) {
    imshow("Image", img);
    waitKey(wait);
}

void Visualizations::set_feasible_set_img(Polygon_2 container) {
    container_inexact = convert_polygon(snap_origin_to_bl(container));
    set_img_params(container_inexact);
    x = h * SCALEFACTOR + 3 * IMG_OFFSET + 2 * MAX_POLY_RADIUS * SCALEFACTOR;
    y = w * SCALEFACTOR + 3 * IMG_OFFSET + 2 * MAX_POLY_RADIUS * SCALEFACTOR;
    outputVideo.open("/Users/kieganlenihan/Desktop/feasible_set.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 100, Size(y, x), true);
}

void Visualizations::draw_feasible_set(Polygon_2 container, Polygon_with_holes_2 sum, Polygon_2 shape) {
    set_output_image(y, x);
    shape_inexact = convert_polygon(shape);
    CGAL::Aff_transformation_2<IK> transform = align_shape(container_inexact, 0, 0, 2 * (IMG_OFFSET + MAX_POLY_RADIUS));
    CGAL::Polygon_2<IK> transformed_cont = transform_shape_IK(container_inexact, transform);
    draw_shape(transformed_cont, COLORS_black, COLORS_white);
    CGAL::Polygon_with_holes_2<IK> sum_inexact = convert_polygon(sum);
    CGAL::Polygon_with_holes_2<IK> transformed_sum = transform_shape_IK(sum_inexact, transform);
    draw_shape(transformed_sum, COLORS_black, COLORS_red);
    Polygon_2 hole;
    Point_2 pt;
    CGAL::Polygon_2<IK> mapped_shape, scaled_shape;
    for (Polygon_with_holes_2::Hole_iterator hi = sum.holes_begin(); hi < sum.holes_end(); ++hi) {
        hole = *hi;
        for (Polygon_2::Vertex_iterator vi = hole.vertices_begin(); vi < hole.vertices_end(); ++vi) {
            pt = *vi;
            CGAL::Aff_transformation_2<IK> map(1, 0, to_inexact(pt.x()), 0, 1, to_inexact(pt.y()), 1);
            mapped_shape = transform_shape_IK(shape_inexact, map);
            scaled_shape = transform_shape_IK(mapped_shape, transform);
            draw_shape(scaled_shape, COLORS_black, COLORS_blue);
        }
    }
    CGAL::Aff_transformation_2<IK> align = align_shape(shape_inexact, 0, 0, IMG_OFFSET);
    CGAL::Polygon_2<IK> aligned_shape = transform_shape_IK(shape_inexact, align);
    draw_shape(aligned_shape, COLORS_black, COLORS_purple);
    outputVideo.write(img);
    visualize_img(10);
}

void Visualizations::draw_starting_point(Polygon_2 container, Polygon_2 shape, Point_2 start, Point_2 finish, Polygon_2 box) {
    set_output_image(y, x);
    shape_inexact = convert_polygon(shape);
    CGAL::Aff_transformation_2<IK> transform = align_shape(container_inexact, 0, 0, 2 * (IMG_OFFSET + MAX_POLY_RADIUS));
    CGAL::Polygon_2<IK> transformed_cont = transform_shape_IK(container_inexact, transform);
    draw_shape(transformed_cont, COLORS_black, COLORS_white);
    CGAL::Polygon_2<IK> mapped_shape, scaled_shape, box_inexact, mapped_start_box, start_zone, mapped_finish_box, end_zone;
    CGAL::Aff_transformation_2<IK> map_start(1, 0, to_inexact(start.x()), 0, 1, to_inexact(start.y()), 1);
    CGAL::Aff_transformation_2<IK> map_finish(1, 0, to_inexact(finish.x()), 0, 1, to_inexact(finish.y()), 1);
    box_inexact = convert_polygon(box);
    mapped_start_box = transform_shape_IK(box_inexact, map_start);
    start_zone = transform_shape_IK(mapped_start_box, transform);
    draw_shape(start_zone, COLORS_black, COLORS_lightgreen);
    mapped_finish_box = transform_shape_IK(box_inexact, map_finish);
    end_zone = transform_shape_IK(mapped_finish_box, transform);
    draw_shape(end_zone, COLORS_black, COLORS_lightcyan);
    mapped_shape = transform_shape_IK(shape_inexact, map_start);
    scaled_shape = transform_shape_IK(mapped_shape, transform);
    draw_shape(scaled_shape, COLORS_black, COLORS_blue);
    // IK::Point_2 start_IK, finish_IK;
    // start_IK = transform_point_IK(to_inexact(start), transform);
    // finish_IK = transform_point_IK(to_inexact(finish), transform);
    // draw_point(start_IK, COLORS_green);
    // draw_point(finish_IK, COLORS_red);
}

void Visualizations::draw_container_sum(Polygon_2 container, Polygon_2 sum) {
    set_output_image(y, x);
    CGAL::Polygon_2<IK> c_inexact = convert_polygon(container);
    CGAL::Aff_transformation_2<IK> transform = align_shape(c_inexact, 0, 0, 2 * (IMG_OFFSET + MAX_POLY_RADIUS));
    CGAL::Polygon_2<IK> transformed_cont = transform_shape_IK(c_inexact, transform);
    draw_shape(transformed_cont, COLORS_black, COLORS_white);
    CGAL::Polygon_2<IK> sum_inexact = convert_polygon(sum);
    CGAL::Polygon_2<IK> transformed_sum = transform_shape_IK(sum_inexact, transform);
    draw_shape(transformed_sum, COLORS_red, COLORS_white);
}

void Visualizations::draw_container_sum(Polygon_2 container, Polygon_with_holes_2 sum) {
    set_output_image(y, x);
    CGAL::Polygon_2<IK> c_inexact = convert_polygon(container);
    CGAL::Aff_transformation_2<IK> transform = align_shape(c_inexact, 0, 0, 2 * (IMG_OFFSET + MAX_POLY_RADIUS));
    CGAL::Polygon_2<IK> transformed_cont = transform_shape_IK(c_inexact, transform);
    draw_shape(transformed_cont, COLORS_black, COLORS_white);
    CGAL::Polygon_with_holes_2<IK> sum_inexact = convert_polygon(sum);
    CGAL::Polygon_with_holes_2<IK> transformed_sum = transform_shape_IK(sum_inexact, transform);
    draw_shape(transformed_sum, COLORS_red, COLORS_white);
}

void Visualizations::draw_minimal_path(Polygon_2 container, Polygon_2 shape, Point_2 start, Point_2 finish, Polygon_2 circ, Polygon_2 OMBB,  Polygon_2 sum, SsPtr iss) {
    set_output_image(y, x);
    shape_inexact = convert_polygon(shape);
    CGAL::Aff_transformation_2<IK> transform = align_shape(container_inexact, 0, 0, 2 * (IMG_OFFSET + MAX_POLY_RADIUS));
    CGAL::Polygon_2<IK> transformed_cont, mapped_shape, scaled_shape, circ_inexact, mapped_start_circ, mapped_finish_circ, start_circ, end_circ, sum_inexact, transformed_sum;
    CGAL::Aff_transformation_2<IK> map_start(1, 0, to_inexact(start.x()), 0, 1, to_inexact(start.y()), 1);
    CGAL::Aff_transformation_2<IK> map_finish(1, 0, to_inexact(finish.x()), 0, 1, to_inexact(finish.y()), 1);
    // Draw
    // container
    transformed_cont = transform_shape_IK(container_inexact, transform);
    draw_shape(transformed_cont, COLORS_black, COLORS_white);
    // iss
    draw_straight_skeleton(*iss, COLORS_purple, transform);
    // sum
    sum_inexact = convert_polygon(sum);
    transformed_sum = transform_shape_IK(sum_inexact, transform);
    draw_shape(transformed_sum, COLORS_red);
    // OMBB
    mapped_shape = transform_shape_IK(shape_inexact, map_start);
    scaled_shape = transform_shape_IK(mapped_shape, transform);
    draw_shape(scaled_shape, COLORS_black, COLORS_blue);
    // start circle
    circ_inexact = convert_polygon(circ);
    mapped_start_circ = transform_shape_IK(circ_inexact, map_start);
    start_circ = transform_shape_IK(mapped_start_circ, transform);
    draw_shape(start_circ, COLORS_black, COLORS_lightgreen);
    // end circle
    mapped_finish_circ = transform_shape_IK(circ_inexact, map_finish);
    end_circ = transform_shape_IK(mapped_finish_circ, transform);
    draw_shape(end_circ, COLORS_black, COLORS_lightcyan);
}