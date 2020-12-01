#pragma once
#include "pch.h"
#include "WeakVisDecomp.h"
#include "IOHandling.h"

void mark_domains(CDT& cdt);

class ShortestPathMap {
public:
	bool useWeakVis;
	Arrangement_2 thePolygon;


	//Used for answering queries
	vector<vector<Path>> pathMap;
	ShortestPathMap();
	ShortestPathMap(Arrangement_2 thePolygon, bool useWeakVis);
	void SingleShortestMap(Arrangement_2& polygon);
};

