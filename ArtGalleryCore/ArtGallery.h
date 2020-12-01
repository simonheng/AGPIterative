#pragma once
//#include "pch.h"
#include <iostream>
#include <vector>

using namespace std;

class ArtGalleryImpl;
//typedef ArtGalleryImpl* AGPTR;
typedef unique_ptr<ArtGalleryImpl> AGPTR;


class ArtGallery {
public:
	int testPolygonID;
	bool useWeakVis;
	int criticalThreshold;
	bool inTestMode; //whether we take input from file or test polygons
	bool inDrawMode; 
	int currentSize; //which test polygon size we are at

	//Saved information for excel sheet output
	int verticesAtEnd = 0;
	int facesAtEnd = 0;
	int criticalVerticesAtEnd = 0;
	int criticalFacesAtEnd = 0;
	int squareSplits = 0;
	int angularSplits = 0;
	int chordSplits = 0;
	int extensionSplits = 0;
	int vislineSplits = 0;
	int solutionSize = 0;
	double preprocessingTime = 0;
	double processingTime = 0;
	int decompositionSize = 0;
	int maxDecompVertices = 0;
	int maxDecompReflexVertices = 0;
	int weakvisLookups = 0;
	int constantLookups = 0;
	int faceVisibilitiesComputed = 0;
	int pointVisibilitiesComputed = 0;
	int currentGranularity = 0;

	//irrational polygon test
	std::vector<double> p5Times = {};
	std::vector<double> hausdorffDistances = {};
	double maxP5Time = 0;
	bool doingP5 = false;

	int intermediateSteps = 0;
	int maxIterations = 200;

	std::string initSVGString = "initial";
	std::string intermediateSVGString = "intermediate-";
	std::string decompSVGString = "decomp";
	std::string criticalSVGString = "critical-";
	std::string polygonName;
	int reflexVertices = 0;
	//etc...


	AGPTR implementation;
	

	//Arrangement_2 theArrangement;
	ArtGallery(int tId, int maxIterations, int criticalThreshold, bool useWeakVis, bool inTestMode = false, bool inDrawMode = true, int currentSize = 20);
	bool initialize(); //placeholder functions
	bool preProcess();
	bool iterations();
	~ArtGallery();
};






