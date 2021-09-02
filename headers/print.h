#ifndef PGN_PRINT_H
#define PGN_PRINT_H

#include <iostream>
#include <vector>

#include <CGAL/General_polygon_2.h>

//-----------------------------------------------------------------------------
// Pretty-print a CGAL polygon. From CGAL source Examples/Minkowski_sum_2
//
template <typename Polygon_2> void print_polygon(const Polygon_2 & pgn)
{
  std::cout << "[ " << pgn.size() << " vertices: (";
  typename Polygon_2::Vertex_const_iterator  vit;
  for (vit = pgn.vertices_begin(); vit != pgn.vertices_end(); ++vit)
    std::cout << "(" << *vit << ')';
  std::cout << ") ]" << std::endl;
}

template <typename Traits>
void print_polygon(const CGAL::General_polygon_2<Traits> & pgn)
{
  std::cout << "[ " << pgn.size() << " curves:" << std::endl;
  typename CGAL::General_polygon_2<Traits>::Curve_const_iterator  cit;
  for (cit = pgn.curves_begin(); cit != pgn.curves_end(); ++cit)
    std::cout << *cit;
  std::cout << " ]" << std::endl;
}

//-----------------------------------------------------------------------------
// Pretty-print a polygon with holes.
//
template <typename Polygon_with_holes>
void print_polygon_with_holes(const Polygon_with_holes & pwh)
{
  if (! pwh.is_unbounded()) {
    std::cout << "{ Outer boundary = "; 
    print_polygon (pwh.outer_boundary());
  }
  else std::cout << "{ Unbounded polygon." << std::endl;

  unsigned int  k = 1;
  typename Polygon_with_holes::Hole_const_iterator hit;
  std::cout << "  " << pwh.number_of_holes() << " holes:" << std::endl;
  for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit, ++k) {
    std::cout << "    Hole #" << k << " = ";
    print_polygon(*hit);
  }
  std::cout << " }" << std::endl;
}

//-----------------------------------------------------------------------------
// Pretty-print a polygon set.
//
template <typename Polygon_set>
void print_polygon_set(const Polygon_set & pgn_set)
{
  typedef typename Polygon_set::Polygon_with_holes_2 Polygon_with_holes;
  typedef std::vector<Polygon_with_holes>            Pgn_with_holes_container;

  Pgn_with_holes_container res(pgn_set.number_of_polygons_with_holes());
  pgn_set.polygons_with_holes(res.begin());
  std::cout << "The result contains " << res.size() << " components:"
            << std::endl;
  typename Pgn_with_holes_container::const_iterator  it;
  for (it = res.begin(); it != res.end(); ++it) {
    std::cout << "--> ";
    print_polygon_with_holes(*it);
  }
}

template<class K>
void print_point ( CGAL::Point_2<K> const& p )
{
  std::cout << "(" << p.x() << "," << p.y() << ")" ;
}

template<class K, class C>
void print_polygon ( CGAL::Polygon_2<K,C> const& poly )
{
  typedef CGAL::Polygon_2<K,C> Polygon ;

  std::cout << "Polygon with " << poly.size() << " vertices" << std::endl ;

  for( typename Polygon::Vertex_const_iterator vi = poly.vertices_begin() ; vi != poly.vertices_end() ; ++ vi )
  {
    print_point(*vi); std::cout << std::endl ;
  }
}

template<class K, class C>
void print_polygons ( std::vector< boost::shared_ptr< CGAL::Polygon_2<K,C> > > const& polies )
{
  typedef std::vector< boost::shared_ptr< CGAL::Polygon_2<K,C> > > PolygonVector ;

  std::cout << "Polygon list with " << polies.size() << " polygons" << std::endl ;

  for( typename PolygonVector::const_iterator pi = polies.begin() ; pi != polies.end() ; ++ pi )
    print_polygon(**pi);
}

template<class K, class C>
void print_polygon_with_holes ( CGAL::Polygon_with_holes_2<K,C> const& polywh )
{
  typedef CGAL::Polygon_with_holes_2<K,C> PolygonWithHoles ;

  std::cout << "Polygon_with_holes having " << polywh.number_of_holes() << " holes" << std::endl ;

  print_polygon(polywh.outer_boundary());

  for( typename PolygonWithHoles::Hole_const_iterator hi = polywh.holes_begin() ; hi != polywh.holes_end() ; ++ hi )
    print_polygon(*hi);
}

template<class K, class C>
void print_polygons_with_holes ( std::vector< boost::shared_ptr< CGAL::Polygon_with_holes_2<K,C> > > const& polies )
{

  typedef std::vector< boost::shared_ptr< CGAL::Polygon_with_holes_2<K,C> > > PolygonWithHolesVector ;

  std::cout << "Polygon_with_holes list with " << polies.size() << " element" << std::endl ;

  for( typename PolygonWithHolesVector::const_iterator pi = polies.begin() ; pi != polies.end() ; ++ pi )
    print_polygon_with_holes(**pi);
}

//-----------------------------------------------------------------------------
// Pretty-print a CGAL straight skeleton. From CGAL source Examples/Straight_skeleton_2
//

template<class K>
void print_straight_skeleton( CGAL::Straight_skeleton_2<K> const& ss )
{
  typedef CGAL::Straight_skeleton_2<K> Ss ;

  typedef typename Ss::Vertex_const_handle     Vertex_const_handle ;
  typedef typename Ss::Halfedge_const_handle   Halfedge_const_handle ;
  typedef typename Ss::Halfedge_const_iterator Halfedge_const_iterator ;

  Halfedge_const_handle null_halfedge ;
  Vertex_const_handle   null_vertex ;

  std::cout << "Straight skeleton with " << ss.size_of_vertices()
            << " vertices, " << ss.size_of_halfedges()
            << " halfedges and " << ss.size_of_faces()
            << " faces" << std::endl ;

  for ( Halfedge_const_iterator i = ss.halfedges_begin(); i != ss.halfedges_end(); ++i )
  {
    print_point(i->opposite()->vertex()->point()) ;
    std::cout << "->" ;
    print_point(i->vertex()->point());
    std::cout << " " << ( i->is_bisector() ? "bisector" : "contour" ) << std::endl;
  }
}

#endif
