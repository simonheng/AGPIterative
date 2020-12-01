#include "pch.h"
#include "IPSolver.h"

IPSolver::IPSolver() {
	model = IloModel(env);
	faceguardVar = IloIntVarArray(env);
	facewitnessVar = IloIntVarArray(env);
	pointguardVar = IloIntVarArray(env);
	faceExpr = IloExprArray(env);
	pointExpr = IloExprArray(env);

	//cplex = IloCplex(model);

	pointConstraints = IloRangeArray(env);
	faceConstraints = IloRangeArray(env);
}

void IPSolver::findSolution(Arrangement_2& thePolygon, bool preferGuards, bool doingP5) {
	IloExpr objective(env);
	double epsilon = 1.0 / (thePolygon.number_of_faces() + thePolygon.number_of_vertices() + 1);

	IloExtractableArray conex(env);

	//Build objective functions and add constraints
	for (auto fit = thePolygon.faces_begin(); fit != thePolygon.faces_end(); ++fit) {
		if (fit->has_outer_ccb()) {
			if (false) {
				objective += faceguardVar[fit->data().id]; //faceguards are same price as pointguards.
				objective += facewitnessVar[fit->data().id] * epsilon; //unseen faces
			}
			else if (preferGuards) {
				objective += faceguardVar[fit->data().id] * (epsilon * 0.9 + 1); //we make faceguards slightly cheaper
				objective += facewitnessVar[fit->data().id] * epsilon; //unseen faces
			}
			else {
				objective += faceguardVar[fit->data().id] * (epsilon + 1);
				objective += facewitnessVar[fit->data().id] * epsilon * 0.9; //unseen faces, slightly cheaper
			}
			
			if (fit->data().isCritical) {

				//if (fit->data().constraintAdded)
					//model.remove(faceConstraints[fit->data().id]);



				faceConstraints[fit->data().id].setExpr(faceExpr[fit->data().id]); //update expressions		

				if (!fit->data().constraintAdded)
					model.add(faceConstraints[fit->data().id]);
				//conex.add(model.add(IloConstraint(faceExpr[fit->data().id] >= 1)));
				//conex.add(model.add(IloRange(faceExpr[fit->data().id] >= 1)));
				fit->data().constraintAdded = true;

			}
		}
	}
	for (auto vit = thePolygon.vertices_begin(); vit != thePolygon.vertices_end(); ++vit) {
		objective += pointguardVar[vit->data().id];
		if (vit->data().isCritical) {
			//	model.remove(pointConstraints[vit->data().id]);

			pointConstraints[vit->data().id].setExpr(pointExpr[vit->data().id]);
			if (!vit->data().constraintAdded)
				model.add(pointConstraints[vit->data().id]);
			//conex.add(model.add(IloRange(faceExpr[fit->data().id] >= 1)));
			vit->data().constraintAdded = true;
		}
	}

	IloExtractable ex = model.add(IloMinimize(env, objective));

	pgVals = IloIntArray(env);
	fgVals = IloIntArray(env);
	fwVals = IloIntArray(env);
	IloCplex cplex = IloCplex(model);
	cplex.setOut(env.getNullStream()); //turning off the damn output

	cplex.solve();
	IloNum sum = cplex.getObjValue();

	//cout << "Actual solution size(IP): " << sum << "\n";

	//reset previous solution
	vertexSolution = {};
	faceSolution = unseenFaces = bothFaces = {};

	cplex.getIntValues(pointguardVar, pgVals);
	cplex.getIntValues(faceguardVar, fgVals);
	cplex.getIntValues(facewitnessVar, fwVals);

	int crisb = 5;

	for (auto vit = thePolygon.vertices_begin(); vit != thePolygon.vertices_end(); ++vit) {
		if(pgVals[vit->data().id] > 0)
			vertexSolution.push_back(vit);
		if (pgVals[vit->data().id] > 1)
			cout << "No way.. jose?";
	}

	for (auto fit = thePolygon.faces_begin(); fit != thePolygon.faces_end(); ++fit) {

		if (fit->has_outer_ccb()) {

			if (fgVals[fit->data().id] > 1)
				cout << "No way.. jorge?";
			if (fwVals[fit->data().id] > 1)
				cout << "No way.. juan?";

			if (fgVals[fit->data().id] > 0) {
				faceSolution.push_back(fit);
				bothFaces.push_back(fit);
			}
			if (fwVals[fit->data().id] > 0) {
				unseenFaces.push_back(fit);
				bothFaces.push_back(fit);
			}
		}
	}

	pgVals.end();
	fgVals.end();
	fwVals.end();

	//clear up variables (prevent memory leak!)
	model.remove(conex);
	conex.end();

	cplex.end();

	model.remove(ex);
	objective.end();
	ex.end();
	objective.end();
}