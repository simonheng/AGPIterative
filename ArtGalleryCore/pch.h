// pch.h: Dies ist eine vorkompilierte Headerdatei.
// Die unten aufgeführten Dateien werden nur einmal kompiliert, um die Buildleistung für zukünftige Builds zu verbessern.
// Dies wirkt sich auch auf die IntelliSense-Leistung aus, Codevervollständigung und viele Features zum Durchsuchen von Code eingeschlossen.
// Die hier aufgeführten Dateien werden jedoch ALLE neu kompiliert, wenn mindestens eine davon zwischen den Builds aktualisiert wird.
// Fügen Sie hier keine Dateien hinzu, die häufig aktualisiert werden sollen, da sich so der Leistungsvorteil ins Gegenteil verkehrt.
#define PCH_H

//check CGAL tss.h if something goes wrong...

// Fügen Sie hier Header hinzu, die vorkompiliert werden sollen.
#pragma once
//general
#include <iostream>

#include <windows.h>	
#include <queue>
#include <string>
#include <random>
#include <unordered_set>
#include <thread>
#include <numeric>
#include <chrono> 
#include <processthreadsapi.h> //measuring time
#include <unordered_map> 

//#include <math.h>
//#include <boost/range.hpp>
//#include <boost/range/join.hpp>
//#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/graphml.hpp> 
//#include <boost/graph/graph_utility.hpp>

//CPLEX
#include <ilcplex/ilocplex.h>

//CGAL
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Quotient.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_walk_along_line_point_location.h>
#include <CGAL/Arr_observer.h>
#include <CGAL/Line_2.h>
#include <CGAL/Ray_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Gps_traits_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/General_polygon_with_holes_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Surface_sweep_2_algorithms.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/enum.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/centroid.h>
#include <CGAL/bounding_box.h>
#include <CGAL/point_generators_2.h>
//#include <CGAL/random_polygon_2.h>
//#include <CGAL/Random.h>
#include <CGAL/algorithm.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangle_2.h>
#include <CGAL/Simple_polygon_visibility_2.h>
//#include "Convex_expansion_visibility_2.h"
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Arr_landmarks_point_location.h>
#include <CGAL/Arr_trapezoid_ric_point_location.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/property_map.h>
#include <CGAL/utils.h>

//Standard
typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
typedef CGAL::Partition_traits_2<K>                         Partition_traits;
typedef Partition_traits::Polygon_2                         Polygon_2;
typedef Partition_traits::Point_2							Point_2;
typedef K::Point_2											KPoint_2;
//typedef Partition_traits::Vector_2									Vector_2;
typedef Partition_traits::Line_2							Line_2;
typedef Partition_traits::Ray_2								Ray_2;
typedef K::Intersect_2										Intersect_2;
typedef CGAL::Arr_segment_traits_2<K>						Traits_2;
typedef Traits_2::Segment_2									Segment_2;

typedef CGAL::Polygon_with_holes_2<K>						Polygon_with_holes_2;

typedef CGAL::Direction_2<K> Direction;

using namespace std;
//Random polygons
//typedef CGAL::Aff_transformation_2<K> Transformation;
//typedef std::list<Point_2>                                Container;
//typedef CGAL::Creator_uniform_2<int, Point_2>             Creator;
//typedef CGAL::Random_points_in_square_2<Point_2, Creator> Point_generator;


//Triangulation
struct Triangle_info
{
	Triangle_info() {}
	int nesting_level;
	int id = -1;
	bool visited = false;

	bool in_domain() {
		return nesting_level % 2 == 1;
	}
};

struct Vertex_T_info
{
	//The index in the original polygon
	int polygonID;
	int triangleID;
};

typedef CGAL::Triangulation_vertex_base_with_info_2<Vertex_T_info, K>	Vb;
typedef CGAL::Triangulation_face_base_with_info_2<Triangle_info, K>		Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>				Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>					TDS;
typedef CGAL::Exact_intersections_tag									Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>		CDT;
typedef CDT::Face_handle												CDT_FH;



typedef CDT::Point														CDT_Point;
typedef CDT::Locate_type												lt;
typedef CGAL::Arr_segment_traits_2<K>::X_monotone_curve_2				X_monotone;
typedef CDT::Finite_faces_iterator										CFFI;




//-------------------------------------------------------------------- ARRANGEMENTS----------------------------------------------------

static char unseenTag = 'u';
static char seenTag = 's';
static char undefinedTag = 'n';

class WeakVisNode;

class Guard {
public:
	Guard() {};
	bool isFace;	
	int id = -1;
	set<int> pweakNodeIds = {};
	//vector<char> seesVertices; //"s = seen, u = unseen"
	
	unordered_map<int, bool> seenVertices;
	unordered_map<int, bool> unseenVertices;

	bool created = false;
	bool isCritical = false;
	bool constraintAdded = false;
	//vector<int> unseenVertexIDs; //use for children faces!
	//vector<int> seenVertexIDs; //use for children faces!
	//IloExtractable conEx;

	//Witness variables!
	//IloExpr expr;
	//IloConstraint seenConstraint;
	//IloIntVar IPvar;
};

class FaceGuard : public Guard
{
public:
	FaceGuard() {};
	//IloIntVar IPfacevar;
	bool isFace = true;
	set<int> fweakNodeIds = {};
};

class PointGuard : public Guard
{
public:
	PointGuard() {};
	bool isFace = false;
	bool isReflex = false;
	bool isOriginal = false;	
};

class Halfedge_info
{
public:
	Halfedge_info() {};
	bool is_diagonal = false;
	bool visited = false;
};


typedef CGAL::Arr_extended_dcel<Traits_2, PointGuard, Halfedge_info, FaceGuard>    Dcel;
typedef CGAL::Arrangement_2<Traits_2, Dcel>											Arrangement_2;
typedef Arrangement_2::Vertex_handle												Vertex_handle;
typedef Arrangement_2::Halfedge_handle												Halfedge_handle;
typedef Arrangement_2::Face_handle													Face_handle;
typedef Arrangement_2::Halfedge_const_handle										Halfedge_const_handle;
typedef Arrangement_2::Edge_const_iterator											Edge_const_iterator;
typedef Arrangement_2::Ccb_halfedge_circulator										Ccb_halfedge_circulator;
typedef CGAL::Arr_extended_dcel_text_formatter<Arrangement_2>						Formatter;
typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>						TEV;
//typedef CGAL::Convex_expansion_visibility_2<Arrangement_2>							CEV;

typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2>						Walk_pl;
typedef CGAL::Arr_landmarks_point_location<Arrangement_2>							Landmarks_pl;
typedef CGAL::Arr_trapezoid_ric_point_location<Arrangement_2>						Trap_pl;


//CONSTANTS
const static vector<string> testPolygonNames = {string("P1"),string("P2"),string("P3"),string("P4"),string("P5"), string("P6"), string("P7")};
const static vector<Point_2> p5Solution = {
		Point_2(3.5 + 5 * std::sqrt(2),	1.5 * std::sqrt(2)),
		Point_2(2, 2 - std::sqrt(2)),
		Point_2(19,1 + (std::sqrt(2)) / 2)};



