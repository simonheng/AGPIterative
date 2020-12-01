#pragma once
#include "pch.h"

class WeakVisNode {
public:
	static int weakVisCounter;


	//The small polygon that his own segment sees
	Arrangement_2 ownPolygon; 
	Polygon_2 boundary;

	int papaId;

	list<shared_ptr<WeakVisNode>> children;
	bool visited = false;
	int id = -1;
	//The polygon obtained by merging with parents and sibblings
	CDT triangulation;
	WeakVisNode();
	WeakVisNode(Arrangement_2& thePolygon, Arrangement_2 previousPolygon, 
		int papaId, Halfedge_handle startingEdge, bool computeWeakvis = true);

	int subTreeSize();
	int maxVertices();
	int maxReflexVertices();
	vector<shared_ptr<WeakVisNode>> subTree();

	bool canSeeDecomp(shared_ptr<WeakVisNode> other);

	void clearTree();
};