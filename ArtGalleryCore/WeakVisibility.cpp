#include "pch.h"
#include "ArrangementFunctions.h"
#include "IOHandling.h"
#include "IPSolver.h"
#include "WeakVisibility.h"
#include "Convex_expansion_visibility_2_temp.h"

typedef Convex_expansion_visibility_2 CEV;

class WeakVisibilityImpl {
public:
	WeakVisibility* wrapper;
	CEV visibility;
	Arrangement_2 polygonBoundary;
	Arrangement_2 weakVisPolygon;


	WeakVisibilityImpl(WeakVisibility* wrapper) {
		this->wrapper = wrapper;
	}

	bool initialize() {
		IOHandling::GetTestPolygon(wrapper->testPolygonID, 20, false, polygonBoundary);
		IOHandling::SetDimensions(polygonBoundary);
		visibility.attach(polygonBoundary);
		
		return IOHandling::DrawWeakVis(visibility, testPolygonNames[wrapper->testPolygonID] + "/" + wrapper->initSVGString);			
	}

	bool compute() {
		vector<Segment_2> testFace;
		if (wrapper->testPolygonID == 0) {
			//diagonal intersects face, but no endpoints in face
			//testFace = { Segment_2(Point_2(35,30), Point_2(53,51)),  Segment_2(Point_2(53,51), Point_2(40,43)),
				//Segment_2(Point_2(40,43), Point_2(35,37)), Segment_2(Point_2(35,37), Point_2(35,30)) };

			//testFace = { Segment_2(Point_2(35,30),Point_2(37,39)),
				//Segment_2(Point_2(37,39), Point_2(35,37)), Segment_2(Point_2(35,37), Point_2(35,30)) };

			//incredibly glitchy face (sees too much):
			/*testFace = { Segment_2(Point_2(40,37),Point_2(40,30)),
				Segment_2(Point_2(40,30), Point_2(48.75, 37)), Segment_2(Point_2(48.75, 37), Point_2(40,37)) };*/

			//something wrong with beta tangents
		/*	testFace = {
				Segment_2(Point_2(29.230769230769234,65),Point_2(16,65)),
				Segment_2(Point_2(16,65), Point_2(15, 48.653846153846160)),
				Segment_2(Point_2(15, 48.653846153846160), Point_2(36,43)),
				Segment_2(Point_2(36,43), Point_2(29.230769230769234,65))				
			};*/

			//something wrong with beta tangents volume 2
			testFace = {
				Segment_2(Point_2(29.230769230769234,65),Point_2(16,65)),
				Segment_2(Point_2(16,65), Point_2(15, 48.653846153846160)),
				Segment_2(Point_2(15, 48.653846153846160), Point_2(36,43)),
				Segment_2(Point_2(36,43), Point_2(29.230769230769234,65))
			};

			//Some strange triangle?
			/*testFace = {
				Segment_2(Point_2(30,35),Point_2(30,37)),
				Segment_2(Point_2(30,37), Point_2(27.647058823529413, 36.176470588235290)),
				Segment_2(Point_2(27.647058823529413, 36.176470588235290), Point_2(30,35))
			};*/

			//Sees too leetle
			/*testFace = {
				Segment_2(Point_2(15,65),Point_2(29.230769230769234, 65)),
				Segment_2(Point_2(29.230769230769234, 65), Point_2(24.615384615384613, 80)),
				Segment_2(Point_2(24.615384615384613, 80), Point_2(15,80)),
				Segment_2(Point_2(15,80), Point_2(15,65))
			};*/

			//large face, diagonal end points in face (boundary)
			//testFace = { Segment_2(Point_2(40,30), Point_2(65,50)), Segment_2(Point_2(65,50), Point_2(36,43)),
			//Segment_2(Point_2(36,43), Point_2(30,37)), Segment_2( Point_2(30,37),Point_2(40,30) )};
		}
		else if (wrapper->testPolygonID == 2) {

			testFace = { Segment_2(Point_2(20,33), Point_2(20,33)),  Segment_2(Point_2(40,33), Point_2(40,42)),
				Segment_2(Point_2(40,42), Point_2(20,42)), Segment_2(Point_2(20,42), Point_2(20,33)) };
		} 
		else if (wrapper->testPolygonID == 3) {
			testFace = {
				Segment_2(Point_2(65,50) ,Point_2(16,65)),
				Segment_2(Point_2(16,65), Point_2(15, 48.653846153846160)),
				Segment_2(Point_2(15, 48.653846153846160), Point_2(65,50)),
			};
		}
		else if (wrapper->testPolygonID == 4) {
			testFace = {
				Segment_2(Point_2(2,0), Point_2(2, -0.63157894736842102)),
				Segment_2(Point_2(2, -0.63157894736842102), Point_2(0, -0.63157894736842102)),
				Segment_2(Point_2(0, -0.63157894736842102), Point_2(0, -0.47058823529411764)),
				Segment_2(Point_2(0, -0.47058823529411764), Point_2(2, 0)),
			};
		}
		else if (wrapper->testPolygonID == 6) { //P7

			testFace = { Segment_2(Point_2(22,42), Point_2(28,42)),  Segment_2(Point_2(28,42), Point_2(38,58)),
				Segment_2(Point_2(38,58), Point_2(12,58)), Segment_2(Point_2(12,58), Point_2(22,42)) };
		}

		visibility.compute_visibility(testFace, weakVisPolygon);

		for (int i = 0; i < visibility.vis_polygons.size(); i++) 
			IOHandling::DrawWeakVis(visibility, testPolygonNames[wrapper->testPolygonID] + "/" + wrapper->intermediateSVGString + to_string(i),
				testFace, {}, i);

		wrapper->intermediateSteps = visibility.vis_polygons.size();

		IOHandling::DrawWeakVis(
			visibility, testPolygonNames[wrapper->testPolygonID] + "/" + wrapper->intermediateSVGString + to_string(wrapper->intermediateSteps),
			testFace, arrangementToPolygon(weakVisPolygon).container());
		wrapper->intermediateSteps++;
		return true;
	}

};

WeakVisibility::WeakVisibility(int tId) {
	this->testPolygonID = tId;
	this->saveIntermediate = saveIntermediate;
	this->implementation = wvPTR(new WeakVisibilityImpl(this));
	//this->implementation = new WeakVisibilityImpl(this);
	this->intermediateSteps = 0;
}

bool WeakVisibility::initialize() {
	return implementation->initialize();
}

bool WeakVisibility::compute() {
	implementation->compute();
	//delete this->implementation;
	return true;
}
