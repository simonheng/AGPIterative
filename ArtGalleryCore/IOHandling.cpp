#include "pch.h"
#include "ArrangementFunctions.h"
#include "Convex_expansion_visibility_2_temp.h"
#include "IOHandling.h"
#include "IPSolver.h"
#include "simpleSVG.hpp"

typedef Convex_expansion_visibility_2 CEV;

static double dimensionX = 1450;
static double dimensionY = 850;
svg::Dimensions dimensions;

static double minX = INT_MAX;
static double minY = INT_MAX;
static double maxX = -INT_MAX;
static double maxY = -INT_MAX;

/// <summary>Reads an input polygon from .pol file </summary>
/// <param name="name">Name of file (randsimple, ortho, etc...)</param> 
/// <param name="n">Size of the polygon</param> 
/// <param name="num">ID of the polygon</param> 
/// <returns>Polygon in terms of a vector of its points</returns> 
std::vector<Point_2> readPoly(std::string name, int n, int num) {
	std::string filename = std::string("c:\\dev\\poly\\" + name + "\\" + name + "-" + std::to_string(n) + "-" + std::to_string(num) + ".pol");
	std::vector<Point_2> poly = {};

	std::ifstream inFile;
	inFile.open(filename);
	std::string str;
	std::getline(inFile, str);
	std::istringstream iss(str);
	std::vector<std::string> results(std::istream_iterator<std::string>{iss},
		std::istream_iterator<std::string>());

	std::string delimiter = "/";
	for (int i = 1; i < results.size(); i = i + 2) {
		double num1, div1;
		double num2, div2;

		int findpos1 = results[i].find(delimiter);
		int findpos2 = results[i + 1].find(delimiter);

		num1 = std::stod(results[i].substr(0, findpos1));
		div1 = std::stod(results[i].substr(findpos1 + 1, results[i].length() - 1 - findpos1));

		num2 = std::stod(results[i + 1].substr(0, findpos2));
		div2 = std::stod(results[i + 1].substr(findpos2 + 1, results[i + 1].length() - 1 - findpos2));

		poly.push_back(Point_2(num1 / div1, num2 / div2));
	}

	/*poly = {Point_2(1,1), Point_2(2, 1), Point_2(2,2)};

	Polygon_2 pol(poly.begin(), poly.end());
	CGAL::Orientation or = pol.orientation();

	std::reverse(poly.begin(), poly.end());*/
	return poly;
}

void IOHandling::GetTestPolygon(int polygonID, int size, bool inTestMode, Arrangement_2& arr) {
	if(inTestMode)
		polygonToArrangement(readPoly("randsimple", size, polygonID), arr);
	else
		polygonToArrangement(testPolygons[polygonID], arr);
}

vector<vector<Point_2>> IOHandling::testPolygons = {
	//p1
	{ Point_2(1,1), Point_2(55,1), Point_2(60,20), Point_2(40,30), Point_2(65,50), Point_2(55,80), Point_2(5,80), Point_2(15,65),
		Point_2(10,50), Point_2(36,43), Point_2(30,37), Point_2(10,30), Point_2(15, 10) },
		//p2
		{ Point_2(9,11), Point_2(67,11), Point_2(61,1), Point_2(75,9), Point_2(77,15), Point_2(67,25), Point_2(51,25),
		Point_2(39,35), Point_2(75,35), Point_2(73,47), Point_2(77,55), Point_2(71,57), Point_2(69,51), Point_2(15,51), Point_2(5, 45),
		Point_2(17,47), Point_2(37,41), Point_2(29,47), Point_2(21,47), Point_2(19,50), Point_2(29,48), Point_2(47,46), Point_2(16, 25),
		Point_2(1, 35), Point_2(1, 1) },
		//p3
		{ Point_2(1,27), Point_2(50,27), Point_2(75,1), Point_2(75,51), Point_2(25,51), Point_2(10,76) },
		//p4
		{ Point_2(-2,55), Point_2(1,50), Point_2(1,1), Point_2(75,1), Point_2(75,50), Point_2(70,60), Point_2(65,50), Point_2(60,60),
		Point_2(55,50), Point_2(25,50),	Point_2(20,60), Point_2(15,50) },
		//p5
		{
		Point_2(0,-12,19),
		Point_2(2 * 19,-12, 19), Point_2(2,0), //first nook		
		Point_2(3,0), Point_2(3,-0.15), Point_2(3.5,0), //second nook
		Point_2(4,0), Point_2(4 * 19,-12,19), Point_2(8 * 19,-18,19), Point_2(8,0), //first bottom rectangle		
		Point_2(12,0), Point_2(12 * 21,-34,21), Point_2(16 * 21,-36,21), Point_2(16,0), //second bottom rectangle
		//Point_2(18.9,0), Point_2(19,-0.5), Point_2(19,0), //third bottom book
		Point_2(20,0), //Point_2(20,0.5), Point_2(30,0.5), Point_2(30,0.6), Point_2(20,0.6), //right tube
		Point_2(20 * 375 ,1776, 375), Point_2(19 * 375, 1776, 375), Point_2(19,4), //first top nook
		Point_2(17 * 6 + 2,4 * 6, 6), Point_2(17 * 6 + 2,4.15 * 6, 6), Point_2(16 * 6 + 5 ,4 * 6, 6), //second top nook
		Point_2(16,4), Point_2(16 * 375 ,1776, 375), Point_2(12 * 375,2486, 375), Point_2(12,4), //first top rectangle
		//Point_2(10.6,4), Point_2(10.6,8), Point_2(10.5,8), Point_2(10.5,4), //top tube
		Point_2(8,4), Point_2(8 * 47,294,47), Point_2(4 * 47,280,47), Point_2(4,4), //second top rectangle
		//Point_2(2.1,4), Point_2(2,4.5), Point_2(2,4), //third top nook
		Point_2(0,4), //Point_2(0,1.8), Point_2(-10,1.8), Point_2(-10,1.7), //left tube
		//Point_2(0,1.7)
		},
		//P6 - false positive witness test
		{
		Point_2(0,0), Point_2(10,0), Point_2(10,11.5), Point_2(15,11.5), Point_2(15,12.5), Point_2(10,12.5), Point_2(10,15), Point_2(30,15),
		Point_2(30,12.5), Point_2(25,12.5), Point_2(25,11.5), Point_2(30,11.5), Point_2(30,0), Point_2(40,0), Point_2(40,20), Point_2(0,20)
		},
		//P7 - weakvis interesction test
		{
			Point_2(0,0), Point_2(50,0),  Point_2(30,10), Point_2(45,20), Point_2(27, 35), Point_2(45, 65), Point_2(5, 65), Point_2(23, 35),
			Point_2(5, 20), Point_2(20,10)
		}
};

svg::Point scalePoint(Point_2 cgalPoint) {
	double xScale = dimensionX / (maxX - minX);
	double yScale = dimensionY / (maxY - minY);
	double finalScale = min(xScale, yScale) * 0.9; 

	return svg::Point(
		20 + finalScale * (CGAL::to_double(cgalPoint.x()) + (-1 * minX)), 
		20 + finalScale * (CGAL::to_double(cgalPoint.y()) + (-1 * minY))
	);
}

svg::Polygon FillFace(Face_handle f, svg::Color stroke, svg::Color fill) {
	svg::Polygon face(svg::Fill(fill), svg::Stroke(1.5, stroke));

	auto eit = f->outer_ccb();


	do {
		face << scalePoint(eit->source()->point());
	} while (++eit != f->outer_ccb());

	return face;
}

void DrawBoundary(Arrangement_2 &theArrangement, svg::Document& doc) {
	svg::Polygon outline(svg::Stroke(1.5, svg::Color::Black));
	Arrangement_2::Ccb_halfedge_circulator eit = *theArrangement.unbounded_face()->inner_ccbs_begin();
	do {
		outline << scalePoint(eit->source()->point());
	} while (++eit != *(theArrangement.unbounded_face()->inner_ccbs_begin()));
	doc << outline;
}

void IOHandling::DrawPolygon(vector<Point_2> polygon, string fileLocation) {
	svg::Document doc("output/" + fileLocation + ".svg", svg::Layout(dimensions, svg::Layout::BottomLeft));

	svg::Polygon outline(svg::Stroke(1.5, svg::Color::Black));
	for(auto it = polygon.begin(); it != polygon.end(); it++)
		outline << scalePoint(*it);
	doc << outline;
	doc.save();
}

void IOHandling::SetDimensions(Arrangement_2& theArrangement) {
	minX = minY = INT_MAX;
	maxX = maxY = -INT_MAX;

	//Find minima and maxima of arrangement
	for (Arrangement_2::Halfedge_handle eit = theArrangement.halfedges_begin(); eit != theArrangement.halfedges_end(); ++eit) {
		double x = CGAL::to_double(eit->source()->point().x());
		double y = CGAL::to_double(eit->source()->point().y());

		if (x < minX)
			minX = x;
		if (y < minY)
			minY = y;
		if (x > maxX)
			maxX = x;
		if (y > maxY)
			maxY = y;
	}
	dimensions = svg::Dimensions(dimensionX, dimensionY);
}


bool IOHandling::DrawDecomposition(Arrangement_2 &theArrangement, shared_ptr<WeakVisNode>& weakVisRoot, string fileLocation) {
	svg::Dimensions dimensions(dimensionX, dimensionY);
	svg::Document doc("output/" + fileLocation + ".svg", svg::Layout(dimensions, svg::Layout::BottomLeft));
	
	//iterate through weak vis decomposition and draw
	stack<shared_ptr<WeakVisNode>> nodeStack;
	stack<bool> colourStack;
	nodeStack.push(weakVisRoot);
	colourStack.push(true);
	//Iterate over weakvisdecomp
	while (!nodeStack.empty())
	{
		shared_ptr<WeakVisNode> currentNode = nodeStack.top();
		bool currentColour = colourStack.top();
		nodeStack.pop();
		colourStack.pop();

		svg::Color fillColour = svg::Color::Green;

		if (!currentColour)
			fillColour = svg::Color::Lime;

		svg::Polygon decompPolygon(svg::Fill(fillColour), svg::Stroke(1.5, svg::Color::Orange));

		//Draw things
		//segments orange
		Arrangement_2::Ccb_halfedge_circulator eit = *currentNode->ownPolygon.unbounded_face()->inner_ccbs_begin();
		do {
			decompPolygon << scalePoint(eit->source()->point());
		} while (++eit != *(currentNode->ownPolygon.unbounded_face()->inner_ccbs_begin()));
		doc << decompPolygon;
		
		//Add children to stack
		for (auto it = currentNode->children.begin(); it != currentNode->children.end(); ++it) {
			nodeStack.push(shared_ptr<WeakVisNode>(*it));
			colourStack.push(!currentColour); //children are opposite colour
		}

	}

	DrawBoundary(theArrangement, doc);
	return doc.save();

}

bool IOHandling::DrawArtGallery(Arrangement_2 &theArrangement, string fileLocation, IPSolver IP) {
	svg::Document doc("output/" + fileLocation + ".svg", svg::Layout(dimensions, svg::Layout::BottomLeft));

	for (auto it = IP.unseenFaces.begin(); it != IP.unseenFaces.end(); ++it) 
		doc << FillFace(*it, svg::Color::Lime, svg::Color::Lime); //lime facewitnesses
	
	for (auto it = IP.faceSolution.begin(); it != IP.faceSolution.end(); ++it) 
		doc << FillFace(*it, svg::Color::Orange, svg::Color::Orange); //orange faceguards
	
	//inner segments gray
	for (auto eit = theArrangement.edges_begin(); eit != theArrangement.edges_end(); ++eit) {
		svg::Line innerSegment(scalePoint(eit->source()->point()), scalePoint(eit->target()->point()),
			svg::Stroke(1, svg::Color::Gray));
		doc << innerSegment;
	}
	DrawBoundary(theArrangement, doc);

	for (auto it = IP.unseenFaces.begin(); it != IP.unseenFaces.end(); ++it) {
		auto eit = (*it)->outer_ccb();
		doc << svg::Elipse(scalePoint(eit->source()->point()), 8, 8, svg::Color::Lime, svg::Stroke(1, svg::Color::Black));
	}

	for (auto it = IP.faceSolution.begin(); it != IP.faceSolution.end(); ++it) {
		auto eit = (*it)->outer_ccb();
		doc << svg::Elipse(scalePoint(eit->source()->point()), 8, 8, svg::Color::Orange, svg::Stroke(1, svg::Color::Black));
	}

	for (auto it = IP.vertexSolution.begin(); it != IP.vertexSolution.end(); ++it)
		doc << svg::Elipse(scalePoint((*it)->point()), 25, 25, svg::Color::Orange, svg::Stroke(1, svg::Color::Black));
	return doc.save();
}

bool IOHandling::DrawCritical(Arrangement_2 &theArrangement, string fileLocation) {
	svg::Document doc("output/" + fileLocation + ".svg", svg::Layout(dimensions, svg::Layout::BottomLeft));

	for (auto fit = theArrangement.faces_begin(); fit != theArrangement.faces_end(); ++fit)
		if(fit->data().isCritical)
			doc << FillFace(fit, svg::Color::Green, svg::Color::Green);

	//inner segments gray
	for (auto eit = theArrangement.edges_begin(); eit != theArrangement.edges_end(); ++eit) {
		svg::Line innerSegment(scalePoint(eit->source()->point()), scalePoint(eit->target()->point()),
			svg::Stroke(0.4, svg::Color::Silver));
		doc << innerSegment;
	}
	DrawBoundary(theArrangement, doc);

	for (auto vit = theArrangement.vertices_begin(); vit != theArrangement.vertices_end(); ++vit)
		if (vit->data().isCritical)
			doc << svg::Elipse(scalePoint((vit)->point()), 25, 25, svg::Color::Green, svg::Stroke(1, svg::Color::Black));

	return doc.save();
}

bool IOHandling::DrawWeakVis(CEV& weakVisibility, string fileLocation, vector<Segment_2> testFace, list<Point_2> weakVis, int step) {
	svg::Document doc("output/" + fileLocation + ".svg", svg::Layout(dimensions, svg::Layout::BottomLeft));
	if (step > -1) 
		 weakVis = weakVisibility.vis_polygons[step];


	//Draw visibility polygon 
	svg::Polygon visPoly(svg::Fill(svg::Color::Lime), svg::Stroke(1, svg::Color::Lime));

	for (auto pit = weakVis.begin(); pit != weakVis.end(); pit++) {
		visPoly << scalePoint(*pit);

	}
	doc << visPoly;
	

	//Draw f0_boundary
	if (weakVisibility.f0_boundary.size() > 0)
	{
		svg::Polygon boundaryPoly(svg::Fill(svg::Color::Green), svg::Stroke(1, svg::Color::Green));
		for (auto pit = weakVisibility.f0_boundary.begin(); pit != weakVisibility.f0_boundary.end(); pit++)
			boundaryPoly << scalePoint(*pit);
		doc << boundaryPoly;
	}

	if (step > -1) {
		Segment_2 alphaTangent = weakVisibility.saved_a_tangents[step];
		Segment_2 betaTangent = weakVisibility.saved_b_tangents[step];
		Segment_2 firstAlphaTangent = weakVisibility.saved_first_a_tangents[step];
		Segment_2 firstBetaTangent = weakVisibility.saved_first_b_tangents[step];

		for(auto pit = weakVisibility.saved_alpha_blockers[step].begin(); pit != weakVisibility.saved_alpha_blockers[step].end(); pit++)
			doc << svg::Elipse(scalePoint(*pit), 25, 25, svg::Color::Red, svg::Stroke(1, svg::Color::Black));
		
		for (auto pit = weakVisibility.saved_beta_blockers[step].begin(); pit != weakVisibility.saved_beta_blockers[step].end(); pit++)
			doc << svg::Elipse(scalePoint(*pit), 25, 25, svg::Color::Blue, svg::Stroke(1, svg::Color::Black));
		

		svg::Polyline alphaLine(svg::Stroke(1.5, svg::Color::Red));
		alphaLine << scalePoint(alphaTangent.source());
		alphaLine << scalePoint(alphaTangent.target());
		doc << alphaLine;

		svg::Polyline betaLine(svg::Stroke(1.5, svg::Color::Blue));
		betaLine << scalePoint(betaTangent.source());
		betaLine << scalePoint(betaTangent.target());
		doc << betaLine;

		svg::Polyline FirstAlphaLine(svg::Stroke(1.5, svg::Color::Silver));
		FirstAlphaLine << scalePoint(firstAlphaTangent.source());
		FirstAlphaLine << scalePoint(firstAlphaTangent.target());
		doc << FirstAlphaLine;

		svg::Polyline firstBetaLine(svg::Stroke(1.5, svg::Color::Silver));
		firstBetaLine << scalePoint(firstBetaTangent.source());
		firstBetaLine << scalePoint(firstBetaTangent.target());
		doc << firstBetaLine;
	}

	//Draw test face 
	svg::Polygon testFacePoly(svg::Fill(svg::Color::Orange), svg::Stroke(5, svg::Color::Orange));
	for (auto sit = testFace.begin(); sit != testFace.end(); sit++) {
		if (sit == testFace.begin())
		{
			doc << svg::Elipse(scalePoint(sit->source()), 25, 25, svg::Color::Black, svg::Stroke(1, svg::Color::Black));
			doc << svg::Elipse(scalePoint(sit->target()), 25, 25, svg::Color::White, svg::Stroke(1, svg::Color::Black));

		}
		testFacePoly << scalePoint(sit->source());
	}
	doc << testFacePoly;

	//inner segments gray
	for (auto eit = weakVisibility.convex_partition.edges_begin(); eit != weakVisibility.convex_partition.edges_end(); ++eit) {
		svg::Line innerSegment(scalePoint(eit->source()->point()), scalePoint(eit->target()->point()),
			svg::Stroke(0.4, svg::Color::Silver));
		doc << innerSegment;
	}
	DrawBoundary(weakVisibility.convex_partition, doc);

	for (auto pit = weakVis.begin(); pit != weakVis.end(); pit++) {
		doc << svg::Elipse(scalePoint(*pit), 5, 5, svg::Color::Lime, svg::Stroke(1, svg::Color::Black));
	}

	return doc.save();
}

chrono::steady_clock::time_point IOHandling::get_cpu_time() {
	//FILETIME a, b, c, d;
	//if (GetProcessTimes(GetCurrentProcess(), &a, &b, &c, &d) != 0) {
	//	//  Returns total user time.
	//	//  Can be tweaked to include kernel times as well.
	//	return
	//		(double)(d.dwLowDateTime |
	//			((unsigned long long)d.dwHighDateTime << 32)) * 0.0000001;
	//}
	//else {
	//	//  Handle error
	//	return 0;
	//}

	return std::chrono::high_resolution_clock::now();
}

 