// OpenCV headers
#include <opencv2/opencv.hpp>           // Mat, circle, imshow, waitKey

// CGAL headers
#include <CGAL/Aff_transformation_2.h>  // transformations
#include <CGAL/Vector_2.h>              // vector

// Working directory headers
#include "path_planner.h"               // type definitions and CGAL Kernels
#include "mover.h"                      // path calculations

// Sub-scopes
using namespace cv;

// Type definitions
typedef CGAL::Aff_transformation_2<EK>      Transformation;
typedef CGAL::Vector_2<EK>                  Vector;
typedef CGAL::Bbox_2                        Bbox_2;
typedef CGAL::Translation                   TRANSLATION;
typedef CGAL::Scaling                       SCALING;

// Constant variables
#define MAX_IMG_DIM 5000
#define IMG_OFFSET 100
#define LINEWIDTH 15
#define n_frames 1000
#define POINT_RADIUS 15

class Visualizations: public Mover {
    private:
        double w, h, SCALEFACTOR;
        int x, y;
        CGAL::Polygon_2<IK> shape_inexact;
        CGAL::Polygon_2<IK> container_inexact;
        Mat img;
    public:
        VideoWriter outputVideo;
        void set_img_params(CGAL::Polygon_2<IK> shape);
        void set_output_image(int x, int y);
        CGAL::Polygon_2<IK> transform_shape_IK(CGAL::Polygon_2<IK> shape, CGAL::Aff_transformation_2<IK> transform);
        CGAL::Polygon_with_holes_2<IK> transform_shape_IK(CGAL::Polygon_with_holes_2<IK> shape, CGAL::Aff_transformation_2<IK> transform);
        IK::Point_2 transform_point_IK(IK::Point_2 pt, CGAL::Aff_transformation_2<IK> transform);
        void draw_shape(CGAL::Polygon_2<IK> shape, Scalar line_color, Scalar fill_color);
        void draw_shape(CGAL::Polygon_with_holes_2<IK> shape, Scalar line_color, Scalar fill_color);
        void draw_point(IK::Point_2 pt, Scalar color);
        CGAL::Aff_transformation_2<IK> align_shape(CGAL::Polygon_2<IK> shape, double x, double y, double shift);
        CGAL::Aff_transformation_2<IK> align_shape(CGAL::Polygon_with_holes_2<IK> shape, double x, double y, double shift);
        CGAL::Polygon_2<IK> convert_polygon(Polygon_2 shape);
        CGAL::Polygon_with_holes_2<IK> convert_polygon(Polygon_with_holes_2 shape);
        void set_feasible_set_img(Polygon_2 container);
        void draw_straight_skeleton(Ss const& iss, Scalar line_color, CGAL::Aff_transformation_2<IK> transform);
        void visualize_img(int wait);
        void draw_feasible_set(Polygon_2 container, Polygon_with_holes_2 sum, Polygon_2 shape);
        void draw_starting_point(Polygon_2 container, Polygon_2 shape, Point_2 start, Point_2 finish, Polygon_2 box);
        void draw_container_sum(Polygon_2 container, Polygon_2 sum);
        void draw_container_sum(Polygon_2 container, Polygon_with_holes_2 sum);
        void draw_minimal_path(Polygon_2 container, Polygon_2 shape, Point_2 start, Point_2 finish, Polygon_2 circ, Polygon_2 OMBB, Polygon_2 sum, SsPtr iss);
};