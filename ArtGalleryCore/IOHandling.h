#pragma once
#include "pch.h"
#include "Convex_expansion_visibility_2_temp.h"
#include "IPSolver.h"
#include "WeakVisDecomp.h"
#include <cstdlib>

//some drawing definitions
typedef Convex_expansion_visibility_2 CEV;

class IOHandling {
public:
	static vector<vector<Point_2>> testPolygons; //the test polygons

	static chrono::steady_clock::time_point get_cpu_time();

	static void GetTestPolygon(int polygonID, int size, bool inTestMode, Arrangement_2& arr);

	static void SetDimensions(Arrangement_2& theArrangement);

	static bool DrawDecomposition(Arrangement_2& theArrangement, shared_ptr<WeakVisNode>& root, string fileLocation);

	static void DrawPolygon(vector<Point_2> polygon, string fileLocation);

	static bool DrawArtGallery(Arrangement_2& theArrangement, string fileLocation, IPSolver IP);

	static bool DrawCritical(Arrangement_2& theArrangement, string fileLocation);

	static bool DrawWeakVis(CEV& weakVisibility, string fileLocation, std::vector<Segment_2> testFace = {}, list<Point_2> weakVis = {}, int step = -1);


};
