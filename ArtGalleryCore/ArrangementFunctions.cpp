#pragma once
#include "pch.h"
#include "ArrangementFunctions.h"

void polygonToArrangement(vector<Point_2> vertices, Arrangement_2& arrangement) {
	vector<Segment_2> edges;
	//Add points to arrangement
	for (vector<Point_2>::iterator it = vertices.begin(); it != vertices.end(); ++it)
		edges.push_back(Segment_2(*it, *((it + 1 == vertices.end()) ? vertices.begin() : it + 1)));

	CGAL::insert_non_intersecting_curves(arrangement, edges.begin(), edges.end());
}

Polygon_2 arrangementToPolygon(Arrangement_2 &arrangement) {
	vector<Point_2> vertices;
	auto eit = *arrangement.unbounded_face()->inner_ccbs_begin();
	do {
		vertices.push_back(eit->source()->point());
	} while (++eit != *arrangement.unbounded_face()->inner_ccbs_begin());

	return Polygon_2(vertices.begin(), vertices.end()); //clockwise order!
}

vector<Segment_2> faceToPolygon(Face_handle face) {
	vector<Segment_2> edges;
	auto eit = face->outer_ccb();
	do {
		edges.push_back(Segment_2(eit->source()->point(), eit->target()->point()));
	} while (++eit != face->outer_ccb());

	return edges; //clockwise order!
}

vector<Point_2> faceToPolygonP(Face_handle face) {
	vector<Point_2> pts;
	auto eit = face->outer_ccb();
	do {
		pts.push_back(eit->source()->point());
	} while (++eit != face->outer_ccb());

	return pts; //clockwise order!
}


//This functions rotates an arrangement by 90 degrees 
Arrangement_2 rotateArrangement(Arrangement_2& original)
{
	vector<Segment_2> rotatedEdges;	
	auto eit = *original.unbounded_face()->inner_ccbs_begin();
	do {
		rotatedEdges.push_back(Segment_2(rotatePointRight(eit->source()->point()), rotatePointRight(eit->target()->point())));
	} 
	while (++eit != *original.unbounded_face()->inner_ccbs_begin());

	Arrangement_2 rotatedArrangement;
	CGAL::insert_non_intersecting_curves(rotatedArrangement, rotatedEdges.begin(), rotatedEdges.end());

	return rotatedArrangement;
}

bool properIntersectFace(Face_handle& face, Ray_2 ray, Segment_2& out) {
	vector<Point_2> inters;

	//Circular do/while loop to find face vertices
	auto eit = face->outer_ccb();
	do {
		Segment_2 edge = Segment_2(eit->source()->point(), eit->target()->point());
		auto interResult =	CGAL::intersection(ray, edge);
		if (interResult) {
			if (const Segment_2* s = boost::get<Segment_2>(&*interResult)) {
				//we are parallel? we are not proper! :(
				return false;
			}
			else {
				Point_2 interPoint = *boost::get<Point_2 >(&*interResult);
				if (interPoint != edge.source()) //we will find it as target
					inters.push_back(interPoint);
			}
			
		}
	} while (++eit != face->outer_ccb());

	if (inters.size() == 2) {
		out = Segment_2(inters[0], inters[1]);
		return true;
	}
	return false;
}

bool properIntersectFace(Face_handle& face, Segment_2 ray, Segment_2& out) {
	vector<Point_2> inters;

	//Circular do/while loop to find face vertices
	auto eit = face->outer_ccb();
	do {
		Segment_2 edge = Segment_2(eit->source()->point(), eit->target()->point());
		auto interResult = CGAL::intersection(ray, edge);
		if (interResult) {
			if (const Segment_2* s = boost::get<Segment_2>(&*interResult)) {
				//we are parallel? we are not proper! :(
				return false;
			}
			else {
				Point_2 interPoint = *boost::get<Point_2 >(&*interResult);
				if (interPoint != edge.source()) //we will find it as target
					inters.push_back(interPoint);
			}

		}
	} while (++eit != face->outer_ccb());

	if (inters.size() == 2) {
		out = Segment_2(inters[0], inters[1]);
		return true;
	}
	return false;
}



//This function shoots a ray up and down and returns the resulting segment
vector<Point_2> shootRayUpDown(Arrangement_2& theArrangement, Point_2 start, bool goUp, bool goDown) {
	//From every reflex vertex, we are going to shoot a ray up and down
	Walk_pl walk_pl(theArrangement);
	vector<Point_2> hitPoints;


	Arrangement_2::Vertex_const_handle v;
	Arrangement_2::Halfedge_const_handle he;
	Point_2 hitPoint;
	while (goDown || goUp) {
		CGAL::Object obj;
		if (goUp) {
			obj = walk_pl.ray_shoot_up(start);
			goUp = false;
		}
		else if (goDown) {
			obj = walk_pl.ray_shoot_down(start);
			goDown = false;
		}

		if (CGAL::assign(v, obj)) {
			hitPoints.push_back(v->point());
		}
		else if (CGAL::assign(he, obj)) {
			//we hit a half-edge
			//check where we intersect the half-edge to find the right y_value
			Halfedge_handle nhe = theArrangement.non_const_handle(he);
			Segment_2 hitCurve = nhe->curve();
			if (!hitCurve.is_vertical()) {
				hitPoints.push_back(Point_2(start.x(), hitCurve.supporting_line().y_at_x(start.x())));
				/*hitVertices.push_back(
					theArrangement.split_edge(nhe, Segment_2(nhe->source()->point(), hitPoint), Segment_2(hitPoint, nhe->target()->point())
				)->target());*/
			}
		}
	}	
	return hitPoints;
}

Vertex_handle splitFaceEdge(Arrangement_2& theArrangement, Point_2 splitter, Face_handle face)
{
	//first identify which edge on the face we are hitting
	auto eit = face->outer_ccb();
	do {
		Segment_2 oldEdge(eit->source()->point(), eit->target()->point());
		if (oldEdge.has_on(splitter)) {
			if (oldEdge.source() == splitter)
				return eit->source();
			if (oldEdge.target() == splitter)
				return eit->target();
			else {
				//we have to split the oldedge
				Halfedge_handle splitEdge = theArrangement.split_edge(eit, Segment_2(oldEdge.source(), splitter), Segment_2(splitter, oldEdge.target()));
				return splitEdge->target();
			}
		}
	} while (++eit != face->outer_ccb());

	return theArrangement.insert_in_face_interior(splitter, face);
}

Ray_2 getRay(int numberOfRays, int rayIndex, Point_2 reflex) {
	int perQuadrant = numberOfRays / 4;
	double offset = (2.0 / perQuadrant) * (rayIndex % perQuadrant);
	if (rayIndex < perQuadrant)
		return Ray_2(reflex, Point_2(reflex.x() - 1 + offset ,reflex.y() + 1));
	else if(rayIndex < perQuadrant * 2)
		return Ray_2(reflex, Point_2(reflex.x() + 1, reflex.y() + 1 - offset));
	else if (rayIndex < perQuadrant * 3)
		return Ray_2(reflex, Point_2(reflex.x() + 1 - offset, reflex.y() - 1));
	else //rayIndex < numberOfRays
		return Ray_2(reflex, Point_2(reflex.x() - 1, reflex.y() - 1 + offset));
}



