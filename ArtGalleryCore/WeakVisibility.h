#pragma once
#include <iostream>
using namespace std;

class WeakVisibilityImpl;


//typedef WeakVisibilityImpl* wvPTR;

typedef shared_ptr<WeakVisibilityImpl> wvPTR;

class WeakVisibility {
public:
	int testPolygonID;
	int intermediateSteps;
	std::string initSVGString = "WVinitial";
	std::string intermediateSVGString = "WVintermediate-";
	std::string decompSVGString = "WVdecomp";

	std::string polygonName;
	bool saveIntermediate;

	wvPTR implementation;

	//Arrangement_2 theArrangement;
	WeakVisibility(int tId);

	bool initialize();
	bool compute();
};