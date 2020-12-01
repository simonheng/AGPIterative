#pragma once
#include "pch.h"
#include "PointFunctions.h"

Point_2 rotatePointRight(Point_2 original) {
	//(x,y) = (y, -x) rotates a point 90 degrees right!
	return Point_2(original.y(), -original.x());
}

Point_2 rotatePointLeft(Point_2 original) {
	//(x,y) = (-y, x) rotates a point 90 degrees left!
	return Point_2(-original.y(), original.x());
}

double Hausdorff(std::vector<Point_2> A, std::vector<Point_2> B) {
	auto h = Segment_2(A[0], B[0]).squared_length();
	h = 0;
	for (int i = 0; i < A.size(); i++) {
		auto shortest = Segment_2(A[i], B[0]).squared_length();
		for (int j = 0; j < B.size(); j++) {
			auto dist = Segment_2(A[i], B[j]).squared_length();
			if (dist < shortest)
				shortest = dist;
		}
		if (shortest > h)
			h = shortest;
	}
	return std::sqrt(CGAL::to_double(h));
}