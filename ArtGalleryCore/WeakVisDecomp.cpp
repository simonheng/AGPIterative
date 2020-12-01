#pragma once
#include "pch.h"
#include "WeakVisDecomp.h"
#include "Visibility.h"
#include "IOHandling.h"

WeakVisNode::WeakVisNode() {
};
int WeakVisNode::weakVisCounter = 0;

WeakVisNode::WeakVisNode(Arrangement_2& thePolygon, Arrangement_2 previousPolygon,
	int papaId, Halfedge_handle startingEdge, bool computeWeakvis) {
	this->papaId = papaId;
	id = WeakVisNode::weakVisCounter;
	WeakVisNode::weakVisCounter++;

	if (computeWeakvis) {
		//compute own visibility polygon
		CEV weakVisibility(previousPolygon);
		Face_handle ownInterior = weakVisibility.compute_visibility({ startingEdge->curve(), startingEdge->curve().flip() }, ownPolygon);
		boundary = arrangementToPolygon(ownPolygon);

		//draw the weakvispolygon
		/*IOHandling::DrawWeakVis(
			weakVisibility, "/tester/wvpoly-" + to_string(id),
			{ startingEdge->curve(), startingEdge->curve().flip() }, boundary.container());*/


		//Find corresponding edge in ownInterior
		Halfedge_handle ownStartingEdge;
		bool startingFound = false;
		auto eit = ownInterior->outer_ccb();
		do {
			if (eit->target()->point() == startingEdge->target()->point() && eit->source()->point() == startingEdge->source()->point()) {
				ownStartingEdge = eit;
				startingFound = true;
			}
		} while (++eit != ownInterior->outer_ccb());

		//find new halfedges added by own polygon
		vector<Segment_2> newEdges = {};
		auto ownEdge = ownStartingEdge;
		auto bigEdge = startingEdge;
		do {
			//still the same
			if (bigEdge->target()->point() == ownEdge->target()->point())
			{
				bigEdge = bigEdge->next();
				ownEdge = ownEdge->next();
			}
			else {
				if (bigEdge->curve().line().has_on(ownEdge->target()->point())) {
					//ownEdge and bigEdge are parallel: The next edge must be new
					ownEdge = ownEdge->next();
				}
				newEdges.push_back(ownEdge->curve());

				bool sameTarget = false;
				bool ownTargetOnEdge = false;

				while (!sameTarget && !ownTargetOnEdge) {
					bigEdge = bigEdge->next();
					sameTarget = bigEdge->target()->point() == ownEdge->target()->point();
					ownTargetOnEdge = Segment_2(bigEdge->source()->point(), bigEdge->target()->point()).has_on(ownEdge->target()->point());
				}

				if (ownTargetOnEdge && !sameTarget) {
					ownEdge = ownEdge->next();
				}
			}

		} while (ownEdge != ownStartingEdge);

		for (auto& newEdge : newEdges)
		{
			//CGAL::insert_point(thePolygon, newEdge.source());
			Face_handle newFace;
			//Because of "interesting CGAL design", we have to find the face we just created...
			CGAL::insert(thePolygon, newEdge);
			Arrangement_2 arrCopy = previousPolygon; //we check previouspolygon and not thePolygon, because this one is way smaller
			CGAL::insert(arrCopy, newEdge);
			//Find face here
			for (auto fit = arrCopy.faces_begin(); fit != arrCopy.faces_end();fit++) {
				if (fit->has_outer_ccb()) {
					bool originalFaceFound = false;
					//try to find the new face
					eit = fit->outer_ccb();
					do {
						Segment_2 seg(eit->curve().source(), eit->curve().target());
						if (seg.has_on(startingEdge->target()->point()) && seg.has_on(startingEdge->source()->point())) {
							originalFaceFound = true;
							break;
						}
					} while (++eit != fit->outer_ccb());
					if (!originalFaceFound) {
						newFace = fit;
						break;
					}
				}
			}

			//Create polygon for child weakvis node
			Arrangement_2 nextPolygon;
			Halfedge_handle childStartingEdge;
			vector<Segment_2> newArrSegs = {};
			//eit = newFace->outer_ccb();
			do {
				if (newEdge.has_on(eit->target()->point()) && newEdge.has_on(eit->source()->point()))
					childStartingEdge = eit;
				newArrSegs.push_back(eit->curve());
			} while (++eit != newFace->outer_ccb());
			CGAL::insert_non_intersecting_curves(nextPolygon, newArrSegs.begin(), newArrSegs.end());

			children.push_back(shared_ptr<WeakVisNode>(new WeakVisNode(thePolygon, nextPolygon, id, childStartingEdge, computeWeakvis)));
		}
	}
	else {
		ownPolygon = thePolygon;
	}

	//find my face in the polygon
	Face_handle myFace;
	boundary = arrangementToPolygon(ownPolygon);

	for (auto fit = thePolygon.faces_begin(); fit != thePolygon.faces_end();fit++) {
		if (fit->has_outer_ccb()) {
			bool faceFound = false;
			//try to find the face
			auto eit = fit->outer_ccb();
			do {
				Segment_2 seg(eit->curve().source(), eit->curve().target());
				if (eit->source()->point() == startingEdge->source()->point() && eit->target()->point() == startingEdge->target()->point()) {
					faceFound = true;
					break;
				}
			} while (++eit != fit->outer_ccb());
			if (faceFound) {
				myFace = fit;
				break;
			}
		}
	}


	myFace->data().fweakNodeIds = { this->id };
	auto eit = myFace->outer_ccb();
	do {
		eit->source()->data().pweakNodeIds.insert(this->id);
	} while (++eit != myFace->outer_ccb());
}

int WeakVisNode::subTreeSize() {
	int result = children.size();

	//Call all children
	for (auto cit = children.begin(); cit != children.end(); cit++)
		result += (*cit)->subTreeSize();

	return result;
}

vector<shared_ptr<WeakVisNode>> WeakVisNode::subTree() {
	vector<shared_ptr<WeakVisNode>> result;

	//Call all children
	for (auto cit = children.begin(); cit != children.end(); cit++) {
		result.push_back(shared_ptr<WeakVisNode>(*cit));

		vector<shared_ptr<WeakVisNode>> childTree = (*cit)->subTree();
		result.insert(result.end(), childTree.begin(), childTree.end());
	}

	return result;
}


int WeakVisNode::maxVertices() {
	int maxVertices = ownPolygon.number_of_vertices();
	for (auto cit = children.begin(); cit != children.end(); cit++) {
		maxVertices = max(maxVertices, (*cit)->maxVertices());
	}
	return maxVertices;
}

int WeakVisNode::maxReflexVertices() {
	//count own reflex vertices
	int maxReflexVertices = 0;
	for (auto vit = ownPolygon.vertices_begin(); vit != ownPolygon.vertices_end(); vit++)
		if (vit->data().isReflex)
			maxReflexVertices++;

	//compare to children
	for (auto cit = children.begin(); cit != children.end(); cit++) {
		maxReflexVertices = max(maxReflexVertices, (*cit)->maxReflexVertices());
	}
	return maxReflexVertices;
}

bool WeakVisNode::canSeeDecomp(shared_ptr<WeakVisNode> other) {
	//is he me, is he my papa, am I his papa, are we sibblings?
	return (this->id == other->id) || (papaId == other->id) || (this->id == other->papaId) || (papaId == other->papaId);	
}

void WeakVisNode::clearTree() {
	for (auto cit = children.begin(); cit != children.end(); cit++) {
		(*cit)->clearTree();
	}

	children = {};
}


