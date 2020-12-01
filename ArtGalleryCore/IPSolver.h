#pragma once
#include "pch.h"

class IPSolver {
public:
	IloModel model;
	//IloCplex cplex;

	IloEnv env;
	vector<Vertex_handle> vertexSolution = {};
	vector<Face_handle> faceSolution = {};
	vector<Face_handle> unseenFaces = {};
	vector<Face_handle> bothFaces = {};

	IloIntVarArray faceguardVar;
	IloIntVarArray facewitnessVar;
	IloIntVarArray pointguardVar;
	IloExprArray faceExpr;
	IloExprArray pointExpr;

	IloRangeArray pointConstraints;
	IloRangeArray faceConstraints;

	IloIntArray pgVals;
	IloIntArray fgVals;
	IloIntArray fwVals;

	IPSolver();
	void findSolution(Arrangement_2& thePolygon, bool preferGuards = false, bool doingP5 = false);

};