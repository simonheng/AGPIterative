#pragma once
#include "pch.h"
#include "Visibility.h"

class Split_observer : public CGAL::Arr_observer<Arrangement_2>
{
public:
	int facesCount = 0;
	bool preprocessingDone = false;

	Visibility* vis;
	Split_observer() {};
	Split_observer(Arrangement_2& arr) : CGAL::Arr_observer<Arrangement_2>(arr) {};	

	virtual void before_split_face(Face_handle f, Halfedge_handle he) {
		//Remove f, make sure to end his values!
		f->data().IPvar.end();
		f->data().IPfacevar.end();
		if (f->data().constraintAdded)
			vis->IP->model.remove(f->data().seenConstraint);
	}

	virtual void after_split_face(Face_handle f1, Face_handle f2, bool is_hole) {
		FaceGuard old = f1->data();
		if (!f1->is_unbounded()) {
			f1->set_data(FaceGuard());
			if (!old.weakNodeIds.empty()) {
				f1->data().weakNodeIds = old.weakNodeIds;
				f2->data().weakNodeIds = old.weakNodeIds;
			}
			f1->data().id = old.id;
			f1->data().isCritical = old.isCritical;
		}
		
		if (preprocessingDone && !f2->is_unbounded()) {
			//Copy necessary things from f1?
			facesCount++;

			if (!f1->is_unbounded()) {
				//If the papa face cannot see it, then the new ones cannot either!
				for (auto it = old.unseenVertexIDs.begin(); it != old.unseenVertexIDs.end(); it++)
				{
					f1->data().seesVertices[*it] = unseenTag;
					f2->data().seesVertices[*it] = unseenTag;
				}
			}
			f1->data().unseenVertexIDs = old.unseenVertexIDs;
			f2->data().unseenVertexIDs = old.unseenVertexIDs;


			f2->data().id = facesCount;		
			vis->addGuard(f1);
			vis->addGuard(f2);
		}
		
	}
	
};
