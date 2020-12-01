#pragma once
#include "pch.h"
#include "ShortestPath.h"

void mark_domains(CDT& ct,
	CDT::Face_handle start,
	int index,
	std::list<CDT::Edge>& border)
{
	if (start->info().nesting_level != -1) {
		return;
	}
	std::list<CDT::Face_handle> queue;
	queue.push_back(start);

	while (!queue.empty()) {
		CDT::Face_handle fh = queue.front();
		queue.pop_front();
		if (fh->info().nesting_level == -1) {
			fh->info().nesting_level = index;
			for (int i = 0; i < 3; i++) {
				CDT::Edge e(fh, i);
				CDT::Face_handle n = fh->neighbor(i);
				if (n->info().nesting_level == -1) {
					if (ct.is_constrained(e)) border.push_back(e);
					else queue.push_back(n);
				}
			}
		}
	}
}
void mark_domains(CDT& cdt)
{
	for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
		it->info().nesting_level = -1;
	}

	int index = 0;
	std::list<CDT::Edge> border;
	mark_domains(cdt, cdt.infinite_face(), index++, border);
	while (!border.empty()) {
		CDT::Edge e = border.front();
		border.pop_front();
		CDT::Face_handle n = e.first->neighbor(e.second);
		if (n->info().nesting_level == -1) {
			mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
		}
	}
}


void ShortestPathMap::SingleShortestMap(Arrangement_2 &polygon) {
	vector<CDT_FH> triangles = {};

	//Create triangulation (and make sure to save polygon IDs)
	CDT T;
	auto eit = *polygon.unbounded_face()->inner_ccbs_begin();
	CDT::Vertex_handle previousVertex = T.insert(eit->source()->point());
	previousVertex->info().polygonID = eit->source()->data().id;
	do {
		CDT::Vertex_handle currentVertex = T.insert(eit->target()->point());
		currentVertex->info().polygonID = eit->target()->data().id;
		
		//This forces the segment to be an edge in the triangulation
		T.insert_constraint(previousVertex, currentVertex);

		previousVertex = currentVertex;
	} while (++eit != *polygon.unbounded_face()->inner_ccbs_begin());
	mark_domains(T);

	int triangleCount = 0;
	for (CFFI it = T.finite_faces_begin();it != T.finite_faces_end();++it) {
		if (it->info().in_domain()) {
			it->info().id =	triangleCount;
			it->vertex(0)->info().triangleID = it->info().id;
			it->vertex(1)->info().triangleID = it->info().id;
			it->vertex(2)->info().triangleID = it->info().id;
			triangleCount++;
			triangles.push_back(it);
		}
	}

	//make shortest path map within our own
	for (auto it = T.all_vertices_begin(); it != T.all_vertices_begin(); ++it)
	{
		stack<CDT_FH> triangleStack;
		triangleStack.push(triangles[it->info().triangleID]);

		//First time no complicated merge: simple segments
		for (int i = 0; i < 3; i++) {
			pathMap[it->info().polygonID][triangleStack.top()->vertex(i)->info().polygonID] =
			{ it->point(), triangleStack.top()->vertex(i)->point() };
		}

		//Iterate over weakvisdecomp
		while (!triangleStack.empty())
		{
			CDT_FH currentTriangle = triangleStack.top();
			triangleStack.pop();

			//outside the polygon or been there already
			if (!currentTriangle->info().in_domain() || currentTriangle->info().visited)
				continue;

			if(currentTriangle->info().id != it->info().triangleID) {
				//Fill the map with funnel merge
				std::vector<int> knownIndices;
				int unknownIndex = -1;

				//identify pre computed shortest_paths 
				for (int i = 0; i < 3; i++) {
					if (pathMap[it->info().polygonID][currentTriangle->vertex(i)->info().polygonID].size() > 0)
						knownIndices.push_back(currentTriangle->vertex(i)->info().polygonID);
					else
						unknownIndex = currentTriangle->vertex(i)->info().polygonID;

					////One funnel is trivial, triangle from set_indices
					//Funnel f1(Path(poly[targ_index], poly[set_indices[0]], {}), Path(poly[targ_index], poly[set_indices[1]], {}));

					////Other funnel is from query point to set_indices
					//Funnel f2(map[query][set_indices[0]], map[query][set_indices[1]]);

					//map[query][targ_index] = f2.Merge(f1);
				}

				//we have already computed this path! (how!?)
				if (unknownIndex == -1)
					continue;
			}

			//been there
			currentTriangle->info().visited = true;

			triangleStack.push(currentTriangle->neighbor(0));
			triangleStack.push(currentTriangle->neighbor(1));
			triangleStack.push(currentTriangle->neighbor(2));	
		}
	}



}

ShortestPathMap::ShortestPathMap() {};
ShortestPathMap::ShortestPathMap(Arrangement_2 thePolygon, bool useWeakVis) {
	//Initialize map
	pathMap = vector<vector<Path>>(thePolygon.number_of_vertices(), vector<Path>(thePolygon.number_of_vertices()));

	//stack<WeakVisNode> nodeStack;
	//nodeStack.push(weakVisRoot);

	////Iterate over weakvisdecomp
	//while (!nodeStack.empty())
	//{
	//	WeakVisNode currentNode = nodeStack.top();
	//	nodeStack.pop();

	//	//add the right things to the map
	//	SingleShortestMap(currentNode.ownPolygon);
	//	currentNode.visited = true;

	//	//Make paths to papa


	//	//Make paths to sibling


	//	//Add children to stack
	//	for (vector<WeakVisNode>::iterator it = currentNode.children.begin(); it != currentNode.children.end(); ++it)
	//		nodeStack.push(*it);
	//	
	//}

}
