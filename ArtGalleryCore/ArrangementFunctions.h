#pragma once
#include "pch.h"
#include "PointFunctions.h"

class Visibility;

//This file includes helper-functions that operate on arrangements.
void polygonToArrangement(vector<Point_2> vertices, Arrangement_2& arrangement);

Polygon_2 arrangementToPolygon(Arrangement_2 &arrangement);
vector<Segment_2> faceToPolygon(Face_handle face);

vector<Point_2> faceToPolygonP(Face_handle face);


Arrangement_2 rotateArrangement(Arrangement_2 &original);

vector<Point_2> shootRayUpDown(Arrangement_2& theArrangement, Point_2 start, bool goUp = true, bool goDown = true);

Vertex_handle splitFaceEdge(Arrangement_2& theArrangement, Point_2 splitter, Face_handle face);

bool properIntersectFace(Face_handle& face, Ray_2 ray, Segment_2& out);
bool properIntersectFace(Face_handle& face, Segment_2 ray, Segment_2& out);

Ray_2 getRay(int numberOfRays, int rayIndex, Point_2 reflex);