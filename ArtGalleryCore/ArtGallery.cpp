#pragma once
#include "pch.h"
#include "ArrangementFunctions.h"
#include "ArtGallery.h"
#include "Convex_expansion_visibility_2_temp.h"
#include "IOHandling.h"
#include "IPSolver.h"
#include "MikkelVisibility.h"
#include "PointFunctions.h"
#include "Visibility.h"
#include "WeakVisDecomp.h"
typedef Convex_expansion_visibility_2 CEV;

class ArtGalleryImpl {
public:
	ArtGallery* wrapper;
	IPSolver IP;
	//vector<Vertex_handle> vertexHandles;

	//Save reflex details, used for splits etc...
	vector<pair<Vertex_handle, Vertex_handle>> reflexNeighbours = {};
	vector<Vertex_handle> reflexHandles = {};
	vector<pair<Vertex_handle, Vertex_handle>> chords = {}; //chords between reflex vertices that can see eachother!

	//Visibility
	unique_ptr<Visibility> vis;
	shared_ptr<WeakVisNode> weakVisRoot;

	int maxGranularity = pow(2, 6); //default value?

	ArtGalleryImpl(ArtGallery* wrapper) {
		vis = unique_ptr<Visibility>(new Visibility());

		this->wrapper = wrapper;		
		//Make sure thePolygon is filled
		IOHandling::GetTestPolygon(wrapper->testPolygonID, wrapper->currentSize, wrapper->inTestMode, vis->polygonBoundary);
		IOHandling::SetDimensions(vis->polygonBoundary);
	
		vis->attach(&IP, wrapper->criticalThreshold);

		IOHandling::GetTestPolygon(wrapper->testPolygonID, wrapper->currentSize, wrapper->inTestMode, vis->thePolygon);

		//Identify reflex vertices
		//use do/while for circular loop
		//The polygon is a "hole" in the unbounded face of the arrangement, thus a clockwise inner_ccb
		auto eit = *vis->thePolygon.unbounded_face()->inner_ccbs_begin();
		do {
			//Assign ids to vertices (this is their polygonal ID, used for the shortest path map later!
			eit->source()->data().isOriginal = true;
			//Left turn, because the boundary is clockwise...
			if (CGAL::orientation(eit->prev()->source()->point(), eit->prev()->target()->point(), eit->target()->point()) == CGAL::LEFT_TURN) {
				//eit->source() is a reflex vertex...
				eit->source()->data().isReflex = true;
				reflexHandles.push_back(eit->source());
				reflexNeighbours.push_back(
					make_pair(eit->prev()->source(), eit->target()
					));

			}
		} while (++eit != *(vis->thePolygon.unbounded_face()->inner_ccbs_begin()));
	}


	bool initialize() {
		auto timeBegin = IOHandling::get_cpu_time();

		Arrangement_2 theRotatedPolygon = rotateArrangement(vis->thePolygon);

		//Shoot orthogonal rays
		for (size_t i = 0; i < reflexHandles.size(); i++) {
			Point_2 prevPoint = reflexNeighbours[i].first->point();
			Point_2 reflexPoint = reflexHandles[i]->point();
			Point_2 nextPoint = reflexNeighbours[i].second->point();
			bool goUp, goDown, goRight, goLeft;
			goUp = goDown = goRight = goLeft = true;

			auto vit = reflexHandles[i]->incident_halfedges();
			do {
				//check for verticalities, because CGAL's shoot_up/shoot_down bugs out otherwise...
				//target is our reflex vertex...
				if (vit->curve().is_vertical()) {
					if (vit->source()->point().y() > reflexPoint.y()) 
						goUp = false;					
					else 
						goDown = false;					
				}
				else if (vit->source()->point().y() == vit->target()->point().y()) {
					if (vit->source()->point().x() > reflexPoint.x()) 
						goRight = false;					
					else 
						goLeft = false;					
				}
			} while (++vit != reflexHandles[i]->incident_halfedges());

			vector<Point_2> newVertices = shootRayUpDown(vis->thePolygon, reflexPoint, goUp, goDown);
			vector<Point_2> rVertices = shootRayUpDown(theRotatedPolygon, rotatePointRight(reflexPoint), goLeft, goRight);
			for (auto &rVertex : rVertices)
				newVertices.push_back(rotatePointLeft(rVertex));

			for (auto newVertex : newVertices)
			{
				//check if the point is contained inside the polygon... 
				//(if it is to the right of one of the two incident reflex segments its okay (right because we found these edges going clockwise))
				if (CGAL::orientation(prevPoint, reflexPoint, newVertex) == CGAL::RIGHT_TURN ||
					CGAL::orientation(reflexPoint, nextPoint, newVertex) == CGAL::RIGHT_TURN) {
					Arrangement_2::X_monotone_curve_2 toInsert(reflexPoint, newVertex);
					
					/*if (toInsert.is_directed_right()) {
						vis->thePolygon.insert_at_left


					}*/
					//TODO: improve insert?

					CGAL::insert(vis->thePolygon, Segment_2(reflexPoint, newVertex));

					//the reflex vertex is the starting point
					//insert into both arrangements
					CGAL::insert(theRotatedPolygon, Segment_2(rotatePointRight(reflexPoint), rotatePointRight(newVertex)));
				}
			}			
		}

		auto timeEnd = IOHandling::get_cpu_time();
		chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(timeEnd - timeBegin);
		wrapper->preprocessingTime += time_span.count();
		timeBegin = IOHandling::get_cpu_time();

		//compute all visibilities that we still have to 
		for (auto vit = vis->thePolygon.vertices_begin(); vit != vis->thePolygon.vertices_end(); ++vit)
			if(!vit->data().created)
				vis->prepareGuard(vit, FaceGuard());

		for (auto vit = vis->thePolygon.vertices_begin(); vit != vis->thePolygon.vertices_end(); ++vit) {
			if (!vit->data().created) {
				vit->data().created = true;
				vis->updateVisibilities(vit);
			}

		}

		for (auto fit = vis->thePolygon.faces_begin(); fit != vis->thePolygon.faces_end(); ++fit)
			if (fit->has_outer_ccb() && !fit->data().created)
				vis->prepareGuard(fit, FaceGuard());

		for (auto fit = vis->thePolygon.faces_begin(); fit != vis->thePolygon.faces_end(); ++fit) {
			if (fit->has_outer_ccb() && !fit->data().created) {
				fit->data().created = true;
				vis->updateVisibilities(fit);
			}
		}

		timeEnd = IOHandling::get_cpu_time();
		time_span = chrono::duration_cast<chrono::duration<double>>(timeEnd - timeBegin);
		wrapper->processingTime += time_span.count();
		//Compute chords for splits!
		for (auto vit = reflexHandles.begin(); vit != reflexHandles.end(); ++vit) {
			//for every other reflex vertex
			for (auto vit2 = vit + 1; vit2 != reflexHandles.end(); ++vit2) {
				//if they see eachother: save reflex chord
				if (vis->seesWitness(*vit, *vit2))
					chords.push_back(make_pair(*vit, *vit2));
			}
		}

		string fileString;
		if(wrapper->inTestMode)
			fileString = to_string(wrapper->currentSize) + "-" + to_string(wrapper->testPolygonID) + "/" + wrapper->initSVGString;
		else
			fileString = testPolygonNames[wrapper->testPolygonID] + "/" + wrapper->initSVGString;


		//returns true if not in Drawmode. Otherwise draws polygon and returns result
		return !wrapper->inDrawMode || IOHandling::DrawArtGallery(vis->thePolygon, fileString, IP);
	}

	bool preProcess() {
		auto timeBegin = IOHandling::get_cpu_time();

		//Set-up weak visibility decomp O(n^2)?)
		auto eit = *vis->thePolygon.unbounded_face()->inner_ccbs_begin(); //random edge

		WeakVisNode::weakVisCounter = 0;

		weakVisRoot = shared_ptr<WeakVisNode>(new WeakVisNode(vis->thePolygon, vis->thePolygon, 0, eit->twin(), wrapper->useWeakVis));

		//build the weak visibility vector for constant look-up
		vis->weakVisNodes.push_back(weakVisRoot);
		vector<shared_ptr<WeakVisNode>> rootSubTree = weakVisRoot->subTree();
		vis->weakVisNodes.insert(vis->weakVisNodes.end(), rootSubTree.begin(), rootSubTree.end());


		auto timeEnd = IOHandling::get_cpu_time();
		chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(timeEnd - timeBegin);
		wrapper->preprocessingTime += time_span.count();

		string fileString;
		if (wrapper->inTestMode)
			fileString = to_string(wrapper->currentSize) + "-" + to_string(wrapper->testPolygonID) + "/" + wrapper->decompSVGString;
		else
			fileString = testPolygonNames[wrapper->testPolygonID] + "/" + wrapper->decompSVGString;

		//returns true if not inDrawMode. Otherwise draws polygon and returns result
		return !wrapper->inDrawMode || IOHandling::DrawDecomposition(vis->thePolygon, weakVisRoot, fileString);
	}

	bool angularSplit(Face_handle& toSplit, int raysStart, bool preferIncident) {
		//choose reflex vertices in random order
		vector<int> indices(reflexHandles.size());
		iota(begin(indices), end(indices), 0);
		random_shuffle(indices.begin(), indices.end());

		bool incidentFound = false;

		for (auto it = indices.begin(); it != indices.end() - 1 && !incidentFound; it++) {
			if (vis->seesWitness(toSplit, reflexHandles[*it])) {
				auto eit = toSplit->outer_ccb();
				do {
					if (eit->source()->data().id == reflexHandles[*it]->data().id) {
						//we are the indicent vertex!
						iter_swap(it, indices.end() - 1);
						incidentFound = true;
					}
				} while (++eit != toSplit->outer_ccb() && !incidentFound);
			}
		}


		if(incidentFound && !preferIncident && indices.size() > 1)
			indices.pop_back();

		int maximumGranularity = maxGranularity;
		int numberOfRays = raysStart;
		while (numberOfRays <= maximumGranularity) {
			bool seenByAReflex = false;

			for (auto it = indices.begin(); it != indices.end(); it++)
			{
				if (vis->seesWitness(toSplit, reflexHandles[*it])) {
					seenByAReflex = true;
					//using the current granularity, try to find a reflex vertex that has at least two intersections!
					Segment_2 firstIntersection;
					int firstIntersectionID = 0;

					//First shoot up
					bool foundIntersection = properIntersectFace(toSplit, getRay(numberOfRays, 0, reflexHandles[*it]->point()), firstIntersection);
					int low = 0;
					int high = numberOfRays - 1;
					int mid;

					//binary search for intersection
					while (!foundIntersection && low <= high) {
						mid = (low + high) / 2;
						Ray_2 midRay = getRay(numberOfRays, mid, reflexHandles[*it]->point());
						foundIntersection = properIntersectFace(toSplit, midRay, firstIntersection);
						if (foundIntersection) {
							firstIntersectionID = mid;
							break; //done
						}

						//Check if we are right or left
						auto eit = toSplit->outer_ccb();
						CGAL::Orientation orCheck;
						do {
							orCheck = midRay.supporting_line().oriented_side(eit->source()->point());
							eit++;
						} while (orCheck == CGAL::COLLINEAR); //if we are collinear check another point :(

						//Are we left of the line?
						if (orCheck == CGAL::LEFT_TURN)
							high = mid - 1;
						else				
							low = mid + 1;
					}

					if (foundIntersection) {
						vector<Segment_2> intersections;
						vector<int> intersectionRayIDs; 

						//look left of firstIntersectionID
						int rayID = (firstIntersectionID + (numberOfRays - 1)) % numberOfRays; //decrement one "on the clock"						
						Segment_2 nextIntersection;
						Ray_2 nextRay = getRay(numberOfRays, rayID, reflexHandles[*it]->point());
						bool weIntersect = properIntersectFace(toSplit, nextRay, nextIntersection);
						while (weIntersect){
							intersections.push_back(nextIntersection);
							intersectionRayIDs.push_back(rayID);
							rayID = (rayID + (numberOfRays - 1)) % numberOfRays; //decrement one "on the clock"
							nextRay = getRay(numberOfRays, rayID, reflexHandles[*it]->point());
							weIntersect = properIntersectFace(toSplit, nextRay, nextIntersection);
						} 
						reverse(intersections.begin(), intersections.end()); //reverse the ones we found so far, they are in the wrong order
						reverse(intersectionRayIDs.begin(), intersectionRayIDs.end());

						intersections.push_back(firstIntersection); //add the first one
						intersectionRayIDs.push_back(firstIntersectionID); 

						//look right of firstIntersectionID
						rayID = (firstIntersectionID + 1) % numberOfRays; //increment one "on the clock"						
						nextRay = getRay(numberOfRays, rayID, reflexHandles[*it]->point());
						weIntersect = properIntersectFace(toSplit, nextRay, nextIntersection);
						while (weIntersect) {
							intersectionRayIDs.push_back(rayID);
							intersections.push_back(nextIntersection);
							rayID = (rayID + 1) % numberOfRays;
							nextRay = getRay(numberOfRays, rayID, reflexHandles[*it]->point());
							weIntersect = properIntersectFace(toSplit, nextRay, nextIntersection);
						}

						if (intersections.size() > 1) {
							int middle = intersections.size() / 2; //integer division rounds down
							Segment_2 middleSegment;
							bool foundIt = true;
							//If we are odd we take the middle one (most central)
							//If we are even, we shoot the middle ray in the next level of granularity, so we double number of rays
							if (intersections.size() % 2 > 0)
								middleSegment = intersections[middle];							
							else 
								foundIt = properIntersectFace(toSplit, getRay(numberOfRays * 2, intersectionRayIDs[middle] * 2 - 1, reflexHandles[*it]->point()), middleSegment);
							
							if (numberOfRays > wrapper->currentGranularity)
								wrapper->currentGranularity = numberOfRays;

							wrapper->angularSplits++;		
					
							vector<Vertex_handle> sources = {};
							vector<Vertex_handle> targets = {};

							for (auto &intersection : intersections) {
								sources.push_back(splitFaceEdge(vis->thePolygon, intersection.source(), toSplit));
								targets.push_back(splitFaceEdge(vis->thePolygon, intersection.target(), toSplit));								
							}


							for (size_t i = 0; i < sources.size(); i++) {
								vis->thePolygon.insert_at_vertices(intersections[i], sources[i], targets[i]);
							}

							return true;
						}
					}
				}
			}

			if (!seenByAReflex)
				cout << "Why????whywhyhw";
			numberOfRays = numberOfRays * 2;
		}

		return false;
	}

	void squareSplit(Face_handle& toSplit) {
		Arrangement_2 onlyFace;
		vector<Point_2> facePts;

		//Take necessary info
		auto eit = toSplit->outer_ccb();
		do {
			facePts.push_back(eit->source()->point());
			CGAL::insert(onlyFace, eit->curve());
		} while (++eit != toSplit->outer_ccb());
		
		
		Arrangement_2 rotatedFace = rotateArrangement(onlyFace);
	
		//Find middle of face and insert as vertex		
		auto bbox = CGAL::bounding_box(facePts.begin(), facePts.end());

		//otherwise we use center of bounding box
		Point_2 middle = Point_2(bbox.xmin() + (bbox.xmax() - bbox.xmin())/2, bbox.ymin() + (bbox.ymax() - bbox.ymin())/2);
		
		Vertex_handle midvertex = splitFaceEdge(vis->thePolygon, middle, toSplit);

		vector<Point_2> newPoints;
		vector<Point_2> horPoints = shootRayUpDown(rotatedFace, rotatePointRight(middle));

		//Rotate them back
		for (auto horPoint : horPoints)
			newPoints.push_back(rotatePointLeft(horPoint));

		vector<Point_2> verPoints = shootRayUpDown(onlyFace, middle);
		newPoints.insert(newPoints.end(), verPoints.begin(), verPoints.end());
	
		vector<Vertex_handle> newVertices;
		for (auto& pit : newPoints) {
			if (middle == pit)
				continue;
			else
				newVertices.push_back(splitFaceEdge(vis->thePolygon, pit, toSplit));
		}

		for (auto& vit : newVertices) 
			vis->thePolygon.insert_at_vertices(Segment_2(middle, vit->point()), midvertex, vit);

		
		wrapper->squareSplits++;
	}

	bool extensionSplit(Face_handle& toSplit) {
		//we will choose randomly!
		vector<int> indices(reflexHandles.size());
		iota(begin(indices), end(indices), 0);
		random_shuffle(indices.begin(), indices.end());

		for (auto it = indices.begin(); it != indices.end(); it++)
		{
			if (vis->seesWitness(toSplit, reflexHandles[*it])) {
				//try to split using both reflex extensions if necessary
				Segment_2 intersections;

				//Because of the OR we only check the second one if we actually need to!
				if (properIntersectFace(toSplit, Ray_2(reflexNeighbours[*it].first->point(), reflexHandles[*it]->point()), intersections))
				{
					//we have found some intersections we can use
					vis->thePolygon.insert_at_vertices(
						intersections,
						splitFaceEdge(vis->thePolygon, intersections.source(), toSplit),
						splitFaceEdge(vis->thePolygon, intersections.target(), toSplit)
					);					
					wrapper->extensionSplits++;
					return true;
				}
				else if (properIntersectFace(toSplit, Ray_2(reflexNeighbours[*it].second->point(), reflexHandles[*it]->point()), intersections)) {
					vis->thePolygon.insert_at_vertices(
						intersections,
						splitFaceEdge(vis->thePolygon, intersections.source(), toSplit),
						splitFaceEdge(vis->thePolygon, intersections.target(), toSplit)
					);			
					wrapper->extensionSplits++;
					return true;
				}
			}
		}
		return false;
	}

	bool chordSplit(Face_handle& toSplit) {
		//choose chords in random order
		vector<int> indices(chords.size());
		iota(begin(indices), end(indices), 0);
		random_shuffle(indices.begin(), indices.end());

		for (auto it = indices.begin(); it != indices.end(); it++)
		{
			if (vis->seesWitness(toSplit, chords[*it].first) && vis->seesWitness(toSplit, chords[*it].second)) {
				//try to split using chord
				Segment_2 intersections;
				if (properIntersectFace(toSplit, Ray_2(chords[*it].first->point(), chords[*it].second->point()), intersections))
				{
					//we have found some intersections we can use
					vis->thePolygon.insert_at_vertices(
						intersections,
						splitFaceEdge(vis->thePolygon, intersections.source(), toSplit),
						splitFaceEdge(vis->thePolygon, intersections.target(), toSplit)
					);	
					wrapper->chordSplits++;
					return true;
				}
				
			}
		}
		return false;
	}

	bool vislineSplit(Face_handle& toSplit, bool isGuard) {
		vector<Vertex_handle> candidates;

		//identify split options
		if (isGuard) {
			for (auto& sv : toSplit->data().seenVertices) {
				if (!vis->seesWitness(vis->allVertices[sv.first], toSplit))
				{
					bool onlyUs = true;
					//we see him but he does not see us (good news)
					//are we the only one that sees him?
					for (auto& vs : IP.vertexSolution)
					{
						if (vis->seesWitness(vs, vis->allVertices[sv.first]))
						{
							onlyUs = false;
							break;
						}
					}

					if (onlyUs) {
						for (auto& fs : IP.faceSolution)
						{
							if (fs->data().id != toSplit->data().id && vis->seesWitness(fs, vis->allVertices[sv.first]))
							{
								onlyUs = false;
								break;
							}
						}
					}
					if (onlyUs) //okay we satisfy all conditions
						candidates.push_back(vis->allVertices[sv.first]);
				}
			}
		}
		else{
			//we want vertex guards that we can see (only those are interesting)
			for (auto& vs : IP.vertexSolution) {
				if (vis->seesWitness(toSplit, vs)) {
					vector<Vertex_handle> seenFVertices = {};
					auto eit = toSplit->outer_ccb();
					do {
						if (vis->seesWitness(vs, eit->source()))
							seenFVertices.push_back(eit->source());
					} while (++eit != toSplit->outer_ccb());

					if (seenFVertices.size() > 2 || (seenFVertices.size() == 2 && !Line_2(vs->point(), seenFVertices[0]->point()).has_on(seenFVertices[1]->point())))
						candidates.push_back(vs);
				}
			}
		}

		if (candidates.size() > 0) {
			//lets go
			//int rI = rand() % candidates.size(); 
			random_shuffle(candidates.begin(), candidates.end());
			vector<pair<Vertex_handle, Vertex_handle>> newVertices;

			for (auto &candidate : candidates) {
				for (auto pedge = vis->pointvisPolygons[candidate->data().id].edges_begin(); pedge != vis->pointvisPolygons[candidate->data().id].edges_end(); pedge++)
				{
					Segment_2 inter;
					if(properIntersectFace(toSplit, *pedge, inter))
						newVertices.push_back(make_pair(splitFaceEdge(vis->thePolygon, inter.source(), toSplit), splitFaceEdge(vis->thePolygon, inter.target(), toSplit)));
				}

				if (!newVertices.empty())
					break;

			}

			if (newVertices.empty())
				return false;
			
			for (auto& newV : newVertices)			
				vis->thePolygon.insert_at_vertices(Segment_2(newV.first->point(), newV.second->point()), newV.first, newV.second);
			
			wrapper->vislineSplits++;
			return true;
		}

		return false;
	}

	bool iterations() {
		auto timeBegin = IOHandling::get_cpu_time();
		auto timePrev = timeBegin;
		wrapper->intermediateSteps = 0;
		do {		
			//split faces if any
			for (auto fit = IP.bothFaces.begin(); fit != IP.bothFaces.end(); ++fit)
			{
				FaceGuard papa = (*fit)->data();				
				bool isGuard = find(IP.unseenFaces.begin(), IP.unseenFaces.end(), *fit) == IP.unseenFaces.end();

				if ((*fit)->data().constraintAdded)
					IP.model.remove(IP.faceConstraints[papa.id]);

				//force it not to see anything
				IloExpr expr(IP.env);
				expr += IP.faceguardVar[papa.id] + IP.facewitnessVar[papa.id];
				IloConstraint c(expr == 0);
				IP.model.add(c);

				int reflexCount = 0;
				//Check some things for split types...

				auto eit = (*fit)->outer_ccb();
				do {
					if (eit->source()->data().isReflex)
						reflexCount++;
				} while (++eit != (*fit)->outer_ccb());

				if (reflexCount > 1 || wrapper->doingP5) //P5 = only squaresplitz
				{
					//Do square split
					squareSplit(*fit);
				}
				else {
					
					int dice = rand() % 10 + 1;
					bool splitDone = false;

					if (dice <= 2) {
						//20% chance of extension split
						splitDone = extensionSplit(*fit);

					}
					else if (dice <= 4) {
						//10% chance of chord split
						splitDone = chordSplit(*fit);

					}
					else if (dice <= 6) {
						//50% of visline split
						splitDone = vislineSplit(*fit, isGuard);
					}

					if (!splitDone) {					//otherwise do angular split!					

						int raysStart = 16;
						splitDone = angularSplit(*fit, raysStart, false);
						//splitDone = true;
						//squareSplit(*fit);

						//try chord and extension first before increasing gran gran
						if (!splitDone)
							splitDone = extensionSplit(*fit);

						if (!splitDone)
							splitDone = chordSplit(*fit);

						while (!splitDone) {
							//here we either mark as unsplittable and use bigIP to continue
							raysStart = maxGranularity;

							//or change maxGran here 
							maxGranularity = maxGranularity * 16;
							splitDone = angularSplit(*fit, raysStart, false);
						}
					}
				}

				//(*fit) is now a new face 
				(*fit)->set_data(FaceGuard());			

				//now we are done splitting...				
				for (auto nvit = vis->thePolygon.vertices_begin(); nvit != vis->thePolygon.vertices_end(); nvit++) 
					if (!nvit->data().created) 
						vis->prepareGuard(nvit, papa);
				
				for (auto nvit = vis->thePolygon.vertices_begin(); nvit != vis->thePolygon.vertices_end(); nvit++) {
					if (!nvit->data().created) {
						nvit->data().created = true;
						vis->updateVisibilities(nvit);
					}
				}

				//handle all new faces and vertices
				for (auto nfit = vis->thePolygon.faces_begin(); nfit != vis->thePolygon.faces_end(); nfit++) 
					if (!nfit->data().created && nfit->has_outer_ccb())
						vis->prepareGuard(nfit, papa);	

				int nfaces = 0;
				for (auto nfit = vis->thePolygon.faces_begin(); nfit != vis->thePolygon.faces_end(); nfit++) {
					if (!nfit->data().created && nfit->has_outer_ccb()) {
						nfaces++;
						nfit->data().created = true;
						vis->updateVisibilities(nfit);
					}
				}
			}

			//Critical loop!
			bool allAreSeen = true;
			bool preferGuards = (wrapper->intermediateSteps % 2) == 0;
			do {
				//Find next solution
				IP.findSolution(vis->thePolygon, preferGuards, wrapper->doingP5);
				if (wrapper->criticalThreshold < 100)
					allAreSeen = vis->verifySolution();
			} while (!allAreSeen);

			string fileString;
			string fileStringCrit;
			string fileStringFace;

			if (wrapper->inDrawMode) {
				if (wrapper->inTestMode) {
					fileString = to_string(wrapper->currentSize) + "-" + to_string(wrapper->testPolygonID) + "/" + wrapper->intermediateSVGString + to_string(wrapper->intermediateSteps);
					fileStringCrit = to_string(wrapper->currentSize) + "-" + to_string(wrapper->testPolygonID) + "/" + wrapper->criticalSVGString + to_string(wrapper->intermediateSteps);
					fileStringFace = to_string(wrapper->currentSize) + "-" + to_string(wrapper->testPolygonID) + "/face_visibility - " + to_string(wrapper->intermediateSteps) + "-";
				}
				else {
					fileString = testPolygonNames[wrapper->testPolygonID] + "/" + wrapper->intermediateSVGString + to_string(wrapper->intermediateSteps);
					fileStringCrit = testPolygonNames[wrapper->testPolygonID] + "/" + wrapper->criticalSVGString + to_string(wrapper->intermediateSteps);
					fileStringFace = testPolygonNames[wrapper->testPolygonID] + "/face_visibility - " + to_string(wrapper->intermediateSteps) + "-";
				}

				//Draw necessary things
				IOHandling::DrawCritical(vis->thePolygon, fileStringCrit);
				IOHandling::DrawArtGallery(vis->thePolygon, fileString, IP);
			}

			auto timeEnd = IOHandling::get_cpu_time();
			chrono::duration<double> time_span2 = chrono::duration_cast<chrono::duration<double>>(timeEnd - timeBegin);

			//Are we P5 (irrational polygon)
			if (wrapper->doingP5) {

				//track cumalative time
				chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(timeEnd - timePrev);
				wrapper->p5Times.push_back(time_span.count());
				timePrev = timeEnd;


				//track Hausdorff.
				vector<Point_2> currentSolution;
				
				for (auto& vertex : IP.vertexSolution)
					currentSolution.push_back(vertex->point());
				
				for (auto& face : IP.faceSolution)
				{
					auto eit = face->outer_ccb();
					do {
						currentSolution.push_back(eit->source()->point());
					} while (++eit != face->outer_ccb());
				}

				//calculate Hausdorff and save...
				wrapper->hausdorffDistances.push_back(Hausdorff(p5Solution, currentSolution));


				if (time_span2.count() >= wrapper->maxP5Time)
					break; //stop iterating!
			}
			else
				if (time_span2.count() >= 500)
					break;


			//print summary
			if (wrapper->inDrawMode) {
				cout << "----------- ITERATION " << wrapper->intermediateSteps << "\n";
				/*cout << "WeakVisLookups so far: " << vis->weakvisLookups << "\n";

				cout << "\n";*/
				int j = 0;
				for (auto fit = IP.faceSolution.begin(); fit != IP.faceSolution.end(); fit++) {
					/*	for (int i = 0; i < vis->weakVisibility.vis_polygons.size(); i++)
							IOHandling::DrawWeakVis(weakVisibility, "100-25/faceguardvis-" + to_string(wrapper->intermediateSteps) + "-" + to_string(i),
								faceToPolygon(f), Arrangement_2(), i);*/



					//draw the weakvispolygon
					/*IOHandling::DrawWeakVis(
						vis->weakVisibility, fileStringFace + to_string(j),
						faceToPolygon(*fit), vis->facevisPolygons[(*fit)->data().id].container());*/
					j++;
				}
			}
			//else
				//cout << " " << wrapper->intermediateSteps << " "; 

			//cout << "Number of vertices: " << vis->thePolygon.number_of_vertices() << " ";
			wrapper->intermediateSteps++;
		} while (wrapper->intermediateSteps < wrapper->maxIterations && IP.bothFaces.size() > 0);
		
		auto timeEnd = IOHandling::get_cpu_time();
		chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(timeEnd - timeBegin);
		wrapper->processingTime += time_span.count();
		//General information
		wrapper->solutionSize = IP.vertexSolution.size();
		wrapper->facesAtEnd = vis->thePolygon.number_of_faces() - 1; //no unbounded face
		wrapper->verticesAtEnd = vis->thePolygon.number_of_vertices();

		//Count critical witnesses
		for (auto vit = vis->thePolygon.vertices_begin(); vit != vis->thePolygon.vertices_end(); vit++)
			if (vit->data().isCritical)
				wrapper->criticalVerticesAtEnd++;
		for (auto fit = vis->thePolygon.faces_begin(); fit != vis->thePolygon.faces_end(); fit++)
			if (fit->data().isCritical)
				wrapper->criticalFacesAtEnd++;

		//Decomp information
		wrapper->decompositionSize = weakVisRoot->subTreeSize() + 1; //this function recursively calculates subtreesize.
		wrapper->maxDecompVertices = weakVisRoot->maxVertices();
		wrapper->maxDecompReflexVertices = weakVisRoot->maxReflexVertices();

		//Visibility computations
		wrapper->constantLookups = vis->constantLookups;
		wrapper->faceVisibilitiesComputed = vis->faceVisibilitiesComputed;
		wrapper->pointVisibilitiesComputed = vis->pointVisibilitiesComputed;
		wrapper->weakvisLookups = vis->weakvisLookups;	

		cout << "Clearing ILO memory\n";

		IP.pointConstraints.end();
		IP.faceConstraints.end();
		IP.pointExpr.end();
		IP.faceExpr.end();
		IP.faceguardVar.end();
		IP.pointguardVar.end();
		IP.facewitnessVar.end();
		IP.model.end();
		IP.env.end();

		vis->polygonBoundary.clear();
		vis->thePolygon.clear();

	


		return true;
	}
};

ArtGallery::ArtGallery(int tId, int maxIterations, int criticalThreshold, bool useWeakVis, bool inTestMode, bool inDrawMode, int currentSize) {
	srand((unsigned int)time(0));
	this->inTestMode = inTestMode;
	this->currentSize = currentSize;
	this->maxIterations = maxIterations;
	this->criticalThreshold = criticalThreshold;
	this->useWeakVis = useWeakVis;
	this->testPolygonID = tId;
	this->inDrawMode = inDrawMode;
	this->implementation = AGPTR(new ArtGalleryImpl(this));
	//this->implementation = ArtGalleryImpl(this);
	
}

bool ArtGallery::initialize() {
	return implementation->initialize();
}

bool ArtGallery::preProcess() {
	return implementation->preProcess();
}

bool ArtGallery::iterations() {
	if (testPolygonID == 3 && currentSize == 1000 && !inDrawMode)
		return true;
	if (testPolygonID == 7 && currentSize == 1000 && !inDrawMode)
		return true;
	if (testPolygonID == 15 && currentSize == 1000 && !inDrawMode)
		return true;
	if (testPolygonID == 17 && currentSize == 1000 && !inDrawMode)
		return true;
	if (testPolygonID == 25 && currentSize == 1000 && !inDrawMode)
		return true;
	if (testPolygonID == 25 && currentSize == 100 && !inDrawMode)
		return true;
	if (testPolygonID == 18 && currentSize == 500)
		return true;


	implementation->iterations();
	return true;
}

ArtGallery::~ArtGallery() {
	//this->implementation->~ArtGalleryImpl();
}