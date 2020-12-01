#pragma once
#include "pch.h"
#include "ArrangementFunctions.h"
#include "Convex_expansion_visibility_2_temp.h"
#include "IOHandling.h"
#include "IPSolver.h"

typedef Convex_expansion_visibility_2 CEV;


class Visibility {
public:
	TEV pointVisibility;
	CEV weakVisibility;
	Arrangement_2 polygonBoundary;
	vector<shared_ptr<WeakVisNode>> weakVisNodes;
	int verticesCount = 0;
	int facesCount = 0;
	int weakvisLookups = 0;
	int constantLookups = 0;
	int faceVisibilitiesComputed = 0;
	int pointVisibilitiesComputed = 0;
	vector<Vertex_handle> allVertices; //lookup array
	vector<Face_handle> allFaces; //lookup array

	Landmarks_pl vertexLocation;
	Arrangement_2 thePolygon;

	//2000 max we allow!
	//Arrangement_2 emptyArr;
	//vector<Arrangement_2> pointvisPolygons = std::vector<Arrangement_2>(2000, emptyArr);
	//vector<Arrangement_2> facevisPolygons = std::vector<Arrangement_2>(2000, emptyArr);

	vector<Polygon_2> pointvisPolygons; 
	vector<Polygon_2> facevisPolygons; 

	//vector<Landmarks_pl> pointPLs = std::vector<Landmarks_pl>(2000, Landmarks_pl());
	//vector<Landmarks_pl> facePLs = std::vector<Landmarks_pl>(2000, Landmarks_pl());

	IPSolver* IP;
	int critProbability;

	//default constructors
	Visibility() {};
	void attach(IPSolver* IP, int critProbability) {
		this->IP = IP;
		this->critProbability = critProbability;
		weakVisibility.attach(polygonBoundary);//only once, otherwise there will be convex decompositions
		pointVisibility.attach(polygonBoundary); //only once, otherwise there will be unecessary triangulations
		vertexLocation.attach(polygonBoundary);



	};

	bool faceSeesWitness(Face_handle faceguard, Vertex_handle witness) {
		if (seesWitness(witness, faceguard)) //if he sees us then we see him
			return true;


		auto eit = faceguard->outer_ccb();
		do{
			if (seesWitness(eit->source(), witness))
				return true;
		} while (++eit != faceguard->outer_ccb());

		faceVisibilitiesComputed++;

		
		CGAL::Bounded_side bside = facevisPolygons[faceguard->data().id].bounded_side(witness->point());
		return bside == CGAL::ON_BOUNDED_SIDE || bside == CGAL::ON_BOUNDARY;

		/*vector<Point_2> facePoly = faceToPolygonP(faceguard);
		list<Point_2> pointVispts = pointvisPolygons[witness->data().id].container();
		return do_intersect(Polygon_2(facePoly.begin(), facePoly.end()), pointvisPolygons[witness->data().id]);*/
		//faceToPolygon(faceGuard);

		eit = faceguard->outer_ccb();
		do {
			for (auto pedge = pointvisPolygons[witness->data().id].edges_begin(); pedge != pointvisPolygons[witness->data().id].edges_end(); pedge++)					
				if (CGAL::do_intersect(Segment_2(pedge->source(), pedge->target()), Segment_2(eit->source()->point(), eit->target()->point())))
					return true;			
		} while (++eit != faceguard->outer_ccb());

		return false;
	}

	template <typename T>
	bool seesWitness(T const& guard, Vertex_handle witness) {	
		int guardID = guard->data().id;
		int witnessID = witness->data().id;

		if (guard->data().seenVertices.find(witnessID) != guard->data().seenVertices.end()){
			//if (is_same<T, Vertex_handle>::value) 
				//witness->data().seenVertices[guardID] = true; //symmetry		
			
			constantLookups++;
			return true;
		}
		if (guard->data().unseenVertices.find(witnessID) != guard->data().unseenVertices.end()) {
			//if (is_same<T, Vertex_handle>::value)
				//witness->data().unseenVertices[guardID] = true; //symmetry	

			constantLookups++;
			return false;
		}

		//first check weakvis decomp
		bool visible = false;
		
		if (guard->data().pweakNodeIds.empty() || witness->data().pweakNodeIds.empty()) {
			cout << "u";
			visible = true;
		}
		for (auto wvit = guard->data().pweakNodeIds.begin();!visible && wvit != guard->data().pweakNodeIds.end();wvit++) {
			for (auto wvit2 = witness->data().pweakNodeIds.begin(); !visible && wvit2 != witness->data().pweakNodeIds.end();wvit2++) {
				visible = weakVisNodes[*wvit]->canSeeDecomp(weakVisNodes[*wvit2]);
			}
		}				
		
		//According to the decomp, it is possible for us to see the witness: go and check
		if (visible) {

			//Check visibility
			CGAL::Bounded_side bside;

			if constexpr (std::is_same<T, Face_handle>::value)
				visible = faceSeesWitness(guard, witness);
			else {
				pointVisibilitiesComputed++;
				bside = pointvisPolygons[guardID].bounded_side(witness->point());
				visible = bside == CGAL::ON_BOUNDED_SIDE || bside == CGAL::ON_BOUNDARY;
			}		
		}
		else {
			weakvisLookups++;
			return false; //don't save in constant lookup!
		}
		if (visible) {
			guard->data().seenVertices[witnessID] = true;

			if (is_same<T, Vertex_handle>::value) 
				witness->data().seenVertices[guardID] = true; //symmetry			
		}
		else {
			guard->data().unseenVertices[witnessID] = true;
			if (is_same<T, Vertex_handle>::value) 
				witness->data().unseenVertices[guardID] = true; //symmetry		
		}

		return visible;
	}
	
	
	template <typename T>
	bool seesWitness(T const& guard, Face_handle witness) {
		bool seen = true;

		//We need to see all vertices!
		auto eit = witness->outer_ccb();
		do {
			if (!seesWitness(guard, eit->source())) {
				seen = false;
				break;
			}	
		} while (++eit != witness->outer_ccb());

			//if (uncreated == verticesNum)
				//cout << "That is really bad \n";

		return seen;
	}

	template <typename T>
	void updateVisibilities(T const& guard, bool onlyUs = false) {
		//Go through all vertices
		for (auto vit = thePolygon.vertices_begin(); vit != thePolygon.vertices_end(); ++vit) {
			if (vit->data().created) {
				if (!onlyUs && vit->data().isCritical && seesWitness(guard, vit)) {
					if (is_same<T, Vertex_handle>::value)
						IP->pointExpr[vit->data().id] += IP->pointguardVar[guard->data().id];
					else
						IP->pointExpr[vit->data().id] += IP->faceguardVar[guard->data().id];
				}
				if (guard->data().isCritical && seesWitness(vit, guard))
					if (is_same<T, Vertex_handle>::value)
						IP->pointExpr[guard->data().id] += IP->pointguardVar[vit->data().id];
					else
						IP->faceExpr[guard->data().id] += IP->pointguardVar[vit->data().id];

			}
		}

		//Go through all faces
		for (auto fit = thePolygon.faces_begin(); fit != thePolygon.faces_end(); ++fit) {
			if (fit->data().created) { //only after preprocessing!
				//check us-them visibility
				if (!onlyUs && fit->data().isCritical && seesWitness(guard, fit))
					if (is_same<T, Vertex_handle>::value)
						IP->faceExpr[fit->data().id] += IP->pointguardVar[guard->data().id];
					else
						IP->faceExpr[fit->data().id] += IP->faceguardVar[guard->data().id];
				if (guard->data().isCritical && seesWitness(fit, guard))
					if (is_same<T, Vertex_handle>::value)
						IP->pointExpr[guard->data().id] += IP->faceguardVar[fit->data().id];
					else
						IP->faceExpr[guard->data().id] += IP->faceguardVar[fit->data().id];
			}
		}
	}

	void prepareGuard(Vertex_handle v, FaceGuard papa) {
		if (v->data().id > -1)
			int cirsb = 15; //shouldnt get doubles

		v->data().id = verticesCount;
		verticesCount++;

		allVertices.push_back(v);

		if (papa.id > -1) {
			v->data().unseenVertices = papa.unseenVertices; //copy unseen table

			//don't copy all of papa's weaknodes, try to locate point inside every weak note
			for (auto& weaknodeId : papa.pweakNodeIds)
			{
				CGAL::Bounded_side bside = weakVisNodes[weaknodeId]->boundary.bounded_side(v->point());
				if (bside == CGAL::ON_BOUNDARY || bside == CGAL::ON_BOUNDED_SIDE)
					v->data().pweakNodeIds.insert(weaknodeId);
			}

		}

		//Part of the initialization... fine, we'll just find the nodes manually
		if (v->data().pweakNodeIds.empty()) {
			for (auto wvit = weakVisNodes.begin(); wvit != weakVisNodes.end(); wvit++)
			{
				CGAL::Bounded_side bside = (*wvit)->boundary.bounded_side(v->point());
				if (bside == CGAL::ON_BOUNDARY || bside == CGAL::ON_BOUNDED_SIDE)
					v->data().pweakNodeIds.insert((*wvit)->id);
			}
		}

		if (v->data().pweakNodeIds.empty())
			cout << "a vertex";

		IP->pointExpr.add(IloExpr(IP->env));
		IP->pointConstraints.add(IloRange(IP->env ,1, IloGetInfinity()));
		IP->pointguardVar.add(IloIntVar(IP->env, 0, 1));


		//Locate vertex (face or halfedge handle)
		//Set up visibility polygon
		CGAL::Arr_point_location_result<Arrangement_2>::Type obj =
			vertexLocation.locate(v->point());
		Arrangement_2::Face_const_handle fLoc;
		Halfedge_const_handle heLoc;
		Arrangement_2::Vertex_const_handle vLoc;
		Arrangement_2 visPoly;

		if (CGAL::assign(fLoc, obj))
		{
			//we are in middle of the main face...
			pointVisibility.compute_visibility(v->point(), fLoc, visPoly);
		}
		else if (CGAL::assign(heLoc, obj)) {
			if (!heLoc->face()->has_outer_ccb())
				heLoc = heLoc->twin();
			//we are in a half-edge
			pointVisibility.compute_visibility(v->point(), heLoc, visPoly);
		}
		else if (CGAL::assign(vLoc, obj)) {
			Halfedge_const_handle he = vLoc->incident_halfedges();
			if (!he->face()->has_outer_ccb())
				he = he->twin()->prev();
			//we are in a vertex
			pointVisibility.compute_visibility(v->point(), he, visPoly);
		}		
		pointvisPolygons.push_back(arrangementToPolygon(visPoly));


		//pointPLs[v->data().id].attach(pointvisPolygons[v->data().id]);

		//Throw dice for critical marking
		int dice = rand() % 100 + 1; //100 sided dice
		if (dice <= critProbability || v->data().id == 1) {
			v->data().isCritical = true;
		}

	}

	void prepareGuard(Face_handle f, FaceGuard papa) {
		f->data().id = facesCount;
		facesCount++;

		allFaces.push_back(f);

		//never copy weaknodes
		//f->data().fweakNodeIds = papa.fweakNodeIds;
		//f->data().pweakNodeIds = papa.pweakNodeIds;
	
		//if our vertices can see it, so can we!
		auto eit = f->outer_ccb();
		do {
			auto vertexWeaknodes = eit->source()->data().pweakNodeIds;
			f->data().pweakNodeIds.insert(vertexWeaknodes.begin(), vertexWeaknodes.end());

			for (auto itr = eit->source()->data().seenVertices.begin(); itr != eit->source()->data().seenVertices.end(); itr++)
			{
				f->data().seenVertices[itr->first] = itr->second;
			}
		} while (++eit != f->outer_ccb());


		//If the papa face cannot see it, then the new ones cannot either!
		f->data().unseenVertices = papa.unseenVertices;

		//initialize vars
		IP->faceExpr.add(IloExpr(IP->env));
		IP->faceguardVar.add(IloIntVar(IP->env, 0, 1));
		IP->facewitnessVar.add(IloIntVar(IP->env, 0, 1));
		IP->faceConstraints.add(IloRange(IP->env, 1, IloGetInfinity()));
		
		//allow us to be critical
		IP->faceExpr[f->data().id] += IP->facewitnessVar[f->data().id];

		//cout << "FaceID " << f->data().id << "\n";

		Arrangement_2 visPoly;
		weakVisibility.compute_visibility(faceToPolygon(f), visPoly);
		facevisPolygons.push_back(arrangementToPolygon(visPoly));
		
		//for (int i = 0; i < weakVisibility.vis_polygons.size(); i++)
			//IOHandling::DrawWeakVis(weakVisibility, "P3/intermediate" + to_string(f->data().id - 1) + "-" + to_string(i),
				//faceToPolygon(f), {}, i);

		//Throw dice for critical marking
		if (rand() % 100 + 1 <= critProbability || f->data().id == 1) {
			f->data().isCritical = true;

			////mark all vertices as critical too, they're free of charge anyway...
			auto eit = f->outer_ccb();
			do {
				if (!eit->source()->data().isCritical) {
					eit->source()->data().isCritical = true;
					updateVisibilities(eit->source(), true); //update the point visibilities too because it is critical now!
				}
			} while (++eit != f->outer_ccb());
		}
		updateVisibilities(f);
	}

	bool verifySolution() {
		bool allVerticesSeen = true;
		bool allFacesSeen = true;

		//Go through all vertices
		for (auto vit = thePolygon.vertices_begin(); vit != thePolygon.vertices_end(); ++vit) {		
			bool vertexSeen = false;

			if (!vit->data().isCritical) {
				//Go through solution vertices
				for (auto solVit = IP->vertexSolution.begin(); solVit != IP->vertexSolution.end(); ++solVit) {
					if (seesWitness(*solVit, vit)) {
						vertexSeen = true;
						break;
					}
				}

				//Go through solution faces
				if (!vertexSeen) {
					for (auto solFit = IP->faceSolution.begin(); solFit != IP->faceSolution.end(); ++solFit) {
						if (seesWitness(*solFit, vit)) {
							vertexSeen = true;
							break;
						}
					}
				}

				if (!vertexSeen) {
					//mark as critical and break
					vit->data().isCritical = true;

					updateVisibilities(vit, true);
					allVerticesSeen = false;
					break;
				}
			}
		}

		//Go through all faces
		for (auto fit = thePolygon.faces_begin(); fit != thePolygon.faces_end(); ++fit) {
			if (fit->has_outer_ccb() && !fit->data().isCritical) {

				bool faceSeen = false;
				//Go through solution vertices
				for (auto solVit = IP->vertexSolution.begin(); solVit != IP->vertexSolution.end(); ++solVit) {
					if (seesWitness(*solVit, fit)) {
						faceSeen = true;
						break;
					}
				}

				//Go through solution faces
				if (!faceSeen) {
					for (auto solFit = IP->faceSolution.begin(); solFit != IP->faceSolution.end(); ++solFit) {
						if (seesWitness(*solFit, fit)) {
							faceSeen = true;
							break;
						}
					}
				}

				if (!faceSeen) {
					//mark all vertices as critical too, they're free of charge anyway...
					auto eit = fit->outer_ccb();
					do {
						eit->source()->data().isCritical = true;
						updateVisibilities(eit->source(), true); //update the point visibilities too because it is critical now!

					} while (++eit != fit->outer_ccb());


					//mark as critical and break
					fit->data().isCritical = true;
					updateVisibilities(fit, true); //make sure onlyUs is on so we don't compute useless visibilities again
					
					
					
					
					allFacesSeen = false;
					break;
				}
			}
		}

		return (allVerticesSeen && allFacesSeen);
	}
};
