#pragma once
#include "pch.h"


static Point_2 empty_point = Point_2(-10000, -10000);

class MikkelVisibility {
public:
	bool same(Point_2 a, Point_2 b) {
		return a == b;
	}
	bool isset(Point_2 p)
	{
		return !same(p, empty_point);
	}
	bool in_between(Segment_2 s, Point_2 a) {
		return s.has_on(a) && !same(a, s.source()) && !same(a, s.target());
	}
	Point_2 point_intersection(Line_2 pq, Segment_2 sgm, int seg_mode = 0) {

		auto result = intersection(pq, sgm);

		if (result) {
			if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
				if (same(s->source(), s->target()))
					return empty_point;
				else if (seg_mode == 1)
					return s->source();
				else if (seg_mode == 2)
					return s->target();
			}
			else {
				Point_2* p = boost::get<Point_2>(&*result);
				return *p;
			}
		}

		return empty_point;
	}

	Point_2 point_intersection(Ray_2 pq, Segment_2 sgm, int seg_mode = 0) {
		if (same(pq.source(), sgm.source()))
			return pq.source();
		if (same(pq.source(), sgm.target()))
			return pq.source();
		//if (pq.has_on(sgm.source()))
			//return sgm.source();
		//if (pq.has_on(sgm.target()))
		//	return sgm.target();

		auto result = intersection(pq, sgm);
		if (result) {
			if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
				if (seg_mode == 1)
					return s->source();
				else if (seg_mode == 2)
					return s->target();
			}
			else {
				Point_2* p = boost::get<Point_2>(&*result);
				return *p;
			}
		}
		return empty_point;
	}

	Point_2 point_intersection(Segment_2 pq, Segment_2 sgm, int seg_mode = 0) {
		auto result = intersection(pq, sgm);

		if (result) {
			if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
				if (seg_mode == 1)
					return s->source();
				else if (seg_mode == 2)
					return s->target();
			}
			else {
				Point_2* p = boost::get<Point_2>(&*result);
				return *p;
			}
		}
		return empty_point;
	}



	int difference_in_polygon(int a, int b, int n) {
		if (a <= b)
		{
			return b - a;
		}
		else
		{
			return n - a + b;
		}
	}


	Point_2 first_intersection(Point_2 p, Point_2 q, std::vector<Point_2> points) {
		Ray_2 pq(p, q);
		Point_2 res = q;
		std::vector<Point_2> inters = {};
		for (int i = 0; i < points.size();i++) {
			int prev = (i - 1 + points.size()) % points.size();
			int next = (i + 1) % points.size();
			int next_next = (next + 1) % points.size();

			Segment_2 curseg(points[i], points[next]);
			Point_2 inter_point = point_intersection(pq, curseg);


			//If q is in the middle of a segment and the next vertex not reflex
			if (in_between(curseg, q))
				return q;

			//if q is a vertex and it is not reflex, there will be no intersection point
			if (points[i] == q && CGAL::orientation(points[prev], points[i], points[next]) == CGAL::LEFT_TURN)
				return q;

			//if q is a vertex and prev is on a different side than next of the ray, there is no intersection
			CGAL::Orientation or1 = CGAL::orientation(p, q, points[next]);
			CGAL::Orientation or2 = CGAL::orientation(p, q, points[prev]);

			if (points[i] == q && or1 != or2 && or1 != CGAL::COLLINEAR && or2 != CGAL::COLLINEAR)
				return q;

			//If q is on the current segment and p is not, and the current segment is colinear with the ray
			bool seg_inside_rayseg = Segment_2(p, q).has_on(points[i]) && Segment_2(p, q).has_on(points[next]);
			if (!curseg.has_on(p) && pq.has_on(points[i]) && pq.has_on(points[next]) && curseg.has_on(q) && !seg_inside_rayseg)
				return q;

			if (isset(inter_point) && !same(inter_point, p) && !same(inter_point, q))
				inters.push_back(inter_point);
		}
		if (inters.size() > 0) {
			res = inters[0];
			for (int i = 1; i < inters.size();i++) {
				if (Segment_2(q, inters[i]).squared_length() < Segment_2(q, res).squared_length())
					res = inters[i];
			}
		}
		return res;
	}

	Arrangement_2 polygon_to_arrangement(std::vector<Point_2> poly) {
		Arrangement_2 arr = Arrangement_2();
		std::vector<Segment_2> segs = {};
		int n = poly.size();

		int crisb = -1;

		//Create segments from the vertices in the polygon
		for (int i = 0; i < poly.size(); i++)
		{
			Segment_2 to_add(poly[i], poly[(i + 1) % n]);
			//segs.push_back(to_add);
			if (segs.size() == 0)
				segs.push_back(to_add);
			else {
				Segment_2 prev = segs[segs.size() - 1];
				if (prev.has_on(poly[(i + 1) % n])) {
					//do not add this segment, and remove previous!
					segs.pop_back();
					to_add = Segment_2(poly[i - 1], poly[(i + 1) % n]);
				}
				else if (to_add.has_on(prev.source())) {
					//do not add this segment, and remove previous!
					segs.pop_back();
					to_add = Segment_2(poly[i - 1], poly[(i + 1) % n]);
				}
				segs.push_back(to_add);
			}
		}
		std::vector<Point_2> new_poly;
		for (int i = 0; i < segs.size(); i++) {
			int next = (i + 1) % segs.size();
			if (!same(segs[next].source(), segs[i].target()))
				throw(std::exception());

			new_poly.push_back(segs[i].source());
		}


		if (!CGAL::is_simple_2(new_poly.begin(), new_poly.end())) {
			throw(std::exception());
		}

		//Insert the segments into the arrangement
		CGAL::insert_non_intersecting_curves(arr, segs.begin(), segs.end());


		return arr;
	}

	std::vector<Point_2> add_points_between(std::vector<Point_2> points, std::vector<Point_2> original_points, int from, int to)
	{
		if (from < to)
			points.insert(points.end(), original_points.begin() + from, original_points.begin() + to);
		else
		{
			points.insert(points.end(), original_points.begin() + from, original_points.end());
			points.insert(points.end(), original_points.begin(), original_points.begin() + to);
		}
		return points;

	}

	std::vector<Point_2> reverse_pgn(std::vector<Point_2> points, int i)
	{
		Point_2 v0 = points[0];
		Point_2 v1 = points[1];

		points.erase(points.begin(), points.begin() + 2);

		points.push_back(v0);
		points.push_back(v1);

		std::reverse(points.begin(), points.end());

		return points;
	}

	std::vector<Point_2> orientate_pgn(std::vector<Point_2> points, int a_index, int b_index)
	{
		std::vector<Point_2> results = {};
		//add a and b
		results.push_back(points[a_index]);
		results.push_back(points[b_index]);

		//add between b and end and between begin and a
		results.insert(results.end(), points.begin() + 1 + b_index, points.end());
		results.insert(results.end(), points.begin(), points.begin() + a_index);

		return results;
	}

	bool in_half_plane(std::vector<CGAL::Orientation> sides, Point_2 p, Point_2 q, Point_2 x)
	{
		bool result = false;
		for (int i = 0; i < sides.size(); i++)
		{
			CGAL::Orientation side = sides[i];
			if (side == CGAL::COLLINEAR)
				result = result || (CGAL::orientation(p, q, x) == side);
			//||
				//	CGAL::orientation(p, q, Point_2(x.x() + alpha, x.y() + alpha)) == side ||
					//CGAL::orientation(p, q, Point_2(x.x() + alpha, x.y() - alpha)) == side ||
					//CGAL::orientation(p, q, Point_2(x.x() - alpha, x.y() - alpha)) == side ||
					//CGAL::orientation(p, q, Point_2(x.x() - alpha, x.y() + alpha)) == side);
			else
				result = result || (CGAL::orientation(p, q, x) == side);
			//CGAL::orientation(p, q, Point_2(x.x() + alpha, x.y() + alpha)) == side &&
			//CGAL::orientation(p, q, Point_2(x.x() + alpha, x.y() - alpha)) == side &&
			//CGAL::orientation(p, q, Point_2(x.x() - alpha, x.y() - alpha)) == side &&
			//CGAL::orientation(p, q, Point_2(x.x() - alpha, x.y() + alpha)) == side);
		}
		return result;
	}

	bool segment_enters_HP_quad(std::vector<Point_2> quad_points, Point_2 p, Point_2 q, Point_2 a, Point_2 b, CGAL::Orientation side)
	{
		std::vector<CGAL::Orientation> sides = { CGAL::LEFT_TURN, CGAL::COLLINEAR };
		bool z = in_half_plane({ side }, p, q, a);
		bool x = in_half_plane({ side }, p, q, b);
		bool y = in_half_plane(sides, quad_points[0], quad_points[1], a);
		bool g = in_half_plane(sides, quad_points[1], quad_points[2], a);
		bool e = in_half_plane(sides, quad_points[2], quad_points[3], a);
		bool f = in_half_plane(sides, quad_points[3], quad_points[0], b);
		bool c = in_half_plane(sides, quad_points[1], quad_points[2], b);
		bool d = in_half_plane(sides, quad_points[2], quad_points[3], b);

		return (
			in_half_plane({ side }, p, q, a) &&
			in_half_plane(sides, quad_points[3], quad_points[0], a) && in_half_plane(sides, quad_points[0], quad_points[1], a) &&
			in_half_plane(sides, quad_points[1], quad_points[2], a) && in_half_plane(sides, quad_points[2], quad_points[3], a)
			) ||
			(
				in_half_plane({ side }, p, q, b) &&
				in_half_plane(sides, quad_points[3], quad_points[0], b) && in_half_plane(sides, quad_points[0], quad_points[1], b) &&
				in_half_plane(sides, quad_points[1], quad_points[2], b) && in_half_plane(sides, quad_points[2], quad_points[3], b)
				);
	}

	std::tuple<Segment_2, int> polygon_exits(Ray_2 pq, std::vector<Point_2> points, Point_2 target, int def)
	{
		std::vector<std::tuple<Segment_2, int>> candidates_and_indices = {};

		std::tuple<Segment_2, int> winner = std::make_tuple(Segment_2(pq.source(), pq.source()), def);

		for (int i = 0; i < points.size(); i++) {
			int prev = i - 1;
			if (i == 0)
				prev = points.size() - 1;

			Segment_2 sgm(points[prev], points[i]);
			Point_2 inter_point = point_intersection(pq, sgm, 2);

			double x = CGAL::to_double(inter_point.x());
			double y = CGAL::to_double(inter_point.y());

			if (isset(inter_point))
			{
				//is s the intersection point? (s = pq.source()), or did we check this already last iteration?
				if (same(pq.source(), inter_point) || same(points[prev], inter_point))
					continue;

				//are we hitting a vertex?
				if (same(inter_point, points[prev]) || same(inter_point, points[i]))
				{
					int next = (i + 1) % points.size();
					CGAL::Orientation or1 = CGAL::orientation(pq.source(), target, points[next]);
					CGAL::Orientation or2 = CGAL::orientation(pq.source(), target, points[prev]);
					//CGAL::Orientation or3 = CGAL::orientation(pq.source(), target, points[i]);

					if (or1 == or2)
					{
						// we never left the polygon
						continue;
					}
					if (or1 == CGAL::COLLINEAR || (or2 == CGAL::COLLINEAR && or1 == CGAL::RIGHT_TURN))
						continue; //i is reflex vertex, we are still in!
				}
				Segment_2 winning_smg = Segment_2(pq.source(), inter_point);

				double x = CGAL::to_double(winning_smg.squared_length());
				double y = CGAL::to_double(std::get<0>(winner).squared_length());

				//We are not intersecting in a vertex and thus leaving the polygon
				if (std::get<1>(winner) == def || winning_smg.squared_length() < std::get<0>(winner).squared_length())
					winner = std::make_tuple(winning_smg, prev);

			}
		}
		return winner;
	}

	std::tuple<int, int> find_rightmost_beam(std::vector<Point_2> points, int i)
	{
		int R = 1, L = i + 1, r = R, l = L, n = points.size() - 1, side = 1;
		std::vector<Point_2> quad_points = { points[0],  points[1],  points[i],  points[i + 1] };
		Polygon_2 quad(quad_points.begin(), quad_points.end());
		Point_2 p = points[1];
		Point_2 q = points[i + 1];
		std::tuple<int, int> null_val = std::make_tuple(0, 0);

		while (r < i || l < n)
		{
			if (side == 1)
			{
				if (r < i) {
					r++;

					//does vr-1vr enter LHP(pq) inter quad?
					if (segment_enters_HP_quad(quad_points, p, q, points[r - 1], points[r], CGAL::LEFT_TURN))
					{
						if (isset(point_intersection(Line_2(points[r - 1], points[r]), Segment_2(points[L], q), 1)))
							return null_val;
						R = r, l = L;
					}
				}
			}
			else {
				if (l < n) {
					l++;
					//does vl-1vl enter RHP(pq) inter quad?
					if (segment_enters_HP_quad(quad_points, p, q, points[l - 1], points[l], CGAL::RIGHT_TURN))
					{
						if (isset(point_intersection(Line_2(points[l - 1], points[l]), Segment_2(points[R], p), 1)))
							return null_val;
						L = l, r = R;
					}
				}
			}

			Point_2 temp_p = point_intersection(Ray_2(points[L], points[R]), Segment_2(points[0], points[1]), 1);
			Point_2 temp_q = point_intersection(Ray_2(points[R], points[L]), Segment_2(points[i], points[i + 1]), 1);
			if (isset(temp_q) && isset(temp_p))
				p = temp_p, q = temp_q;
			else
				return null_val;
			side = -side;
		}

		Line_2 v0v1(points[0], points[1]);
		Line_2 vivi1(points[i], points[i + 1]);

		//Is pq in LHP v0v1 and LHP vivi1 (so within the polygon)?
		bool is_q_good = in_half_plane({ CGAL::LEFT_TURN, CGAL::COLLINEAR }, points[0], points[1], q);
		bool is_p_good = in_half_plane({ CGAL::LEFT_TURN, CGAL::COLLINEAR }, points[i], points[i + 1], p);
		bool is_vi1_good = in_half_plane({ CGAL::LEFT_TURN, CGAL::COLLINEAR }, points[0], points[1], points[i + 1]);
		bool is_v1_good = in_half_plane({ CGAL::LEFT_TURN, CGAL::COLLINEAR }, points[i], points[i + 1], points[1]);

		is_vi1_good = is_vi1_good || !in_half_plane({ CGAL::RIGHT_TURN }, points[0], points[1], points[i + 1]);
		is_v1_good = is_vi1_good || !in_half_plane({ CGAL::RIGHT_TURN }, points[i], points[i + 1], points[1]);

		//return std::make_tuple(R, L);

		if ((is_q_good && is_p_good) || (is_vi1_good && is_v1_good))
			return std::make_tuple(R, L);
		else
			return null_val;


	}

	Segment_2 point_to_edge_visibility(std::vector<Point_2> points, Point_2 p)
	{
		Segment_2 empty_segment = Segment_2(empty_point, empty_point);
		int L = 0, R = 1, n = points.size();
		Point_2 a = points[0], b = points[1];

		//is p in LHP(v0v1)? if not, this subroutine does not work.
		if (!in_half_plane({ CGAL::LEFT_TURN, CGAL::COLLINEAR }, a, b, p))
			return empty_segment;


		for (int i = 1; i < n - 1; i++)
		{

			Point_2 i1 = points[(i + 1) % n];
			Segment_2 vivi1(points[i], i1);

			//triangle check
			if (in_half_plane({ CGAL::LEFT_TURN }, a, b, i1) && in_half_plane({ CGAL::LEFT_TURN }, b, p, i1) && in_half_plane({ CGAL::LEFT_TURN }, p, a, i1))
			{
				//if vivi1 crosses ap left to right (if there is an intersection, and if vi is in LHP(ap) and vi1 is in RHP(ap)
				if (in_half_plane({ CGAL::LEFT_TURN, CGAL::COLLINEAR }, a, p, points[i]) && in_half_plane({ CGAL::RIGHT_TURN, CGAL::COLLINEAR }, a, p, i1) && isset(point_intersection(vivi1, Segment_2(a, p))))
				{

					//is vi1 in RHP(bp)?
					if (in_half_plane({ CGAL::RIGHT_TURN, CGAL::COLLINEAR }, b, p, i1))
						return empty_segment;
					else
					{
						Point_2 pi = point_intersection(Ray_2(p, i1), Segment_2(points[0], points[1]));
						if (isset(pi)) {
							L = (i + 1) % n;
							a = point_intersection(Ray_2(p, i1), Segment_2(points[0], points[1]));
						}
					}
				}

				//if vivi1 crosses bp right to left (if there is an intersection, and if vi is in RHP(bp) and vi1 is in LHP(bp)
				if (in_half_plane({ CGAL::RIGHT_TURN, CGAL::COLLINEAR }, b, p, points[i]) && in_half_plane({ CGAL::LEFT_TURN, CGAL::COLLINEAR }, b, p, i1) && isset(point_intersection(vivi1, Segment_2(b, p))))
				{
					//is vi1 in LHP(ap)?
					if (in_half_plane({ CGAL::LEFT_TURN, CGAL::COLLINEAR }, a, p, i1))
						return empty_segment;
					else
					{
						Point_2 pi = point_intersection(Ray_2(p, i1), Segment_2(points[0], points[1]));
						if (isset(pi)) {
							R = (i + 1) % n;
							b = pi;
						}
					}
				}
			}
		}

		return Segment_2(a, b);
	}

	std::vector<std::vector<Point_2>> split_for_vis(Point_2 a, Point_2 b, std::vector<Point_2>& original_points) {
		//find first intersections
		Point_2 a_prime = first_intersection(b, a, original_points);
		Point_2 b_prime = first_intersection(a, b, original_points);
		int a_index = -1, b_index = -1, a_prime_index = -1, b_prime_index = -1;
		bool a_b_col_seg = false;
		std::vector<int> vertices_hit = {};

		//pre process, add a,b, a_exit_point and b_exit_point as vertices to polygon!
		for (int i = 0; i < original_points.size(); i++) {
			Point_2 cur_point = original_points[i];
			Point_2 next_point = original_points[(i + 1) % original_points.size()];
			Segment_2 cur_seg(cur_point, next_point);
			//if(cur_seg.collinear_has_on(a) && cur_seg.collinear_has_on

			//time saver?
			if (a_index > -1 && b_index > -1 && a_prime_index > -1 && b_prime_index > -1)
				break;

			//Vertex checks
			if (same(cur_point, a))  //a is at this vertex
				a_index = i;
			else if (same(cur_point, b)) //b is at this vertex
				b_index = i;
			if (same(cur_point, a_prime))  //a exits at this vertex
				a_prime_index = i;
			else if (same(cur_point, b_prime)) //b exits at vertex
				b_prime_index = i;

			//Segment checks
			bool a_between = in_between(cur_seg, a);
			bool a_x_between = in_between(cur_seg, a_prime);
			bool b_between = in_between(cur_seg, b);
			bool b_x_between = in_between(cur_seg, b_prime);

			if (a_between || a_x_between) {
				if (a_between && a_x_between) {
					//both, add only one vertex
					auto itPos = original_points.begin() + (i + 1);
					original_points.insert(itPos, a);
					a_index = i + 1;
					a_prime_index = i + 1;
				}
				else if (a_between) {
					auto itPos = original_points.begin() + (i + 1);
					original_points.insert(itPos, a);
					a_index = i + 1;
				}
				else {
					auto itPos = original_points.begin() + (i + 1);
					original_points.insert(itPos, a_prime);
					a_prime_index = i + 1;
				}
			}
			else if (b_between || b_x_between) {
				//add point in between
				if (b_between && b_x_between) {
					//both, add only one vertex
					auto itPos = original_points.begin() + (i + 1);
					original_points.insert(itPos, b);
					b_index = i + 1;
					b_prime_index = i + 1;
				}
				else if (b_between) {
					auto itPos = original_points.begin() + (i + 1);
					original_points.insert(itPos, b);
					b_index = i + 1;
				}
				else {
					auto itPos = original_points.begin() + (i + 1);
					original_points.insert(itPos, b_prime);
					b_prime_index = i + 1;
				}
			}
			if (in_between(Segment_2(a, b), cur_point))
				vertices_hit.push_back(i);
		}

		/*if (a_prime_index == -1)
			a_prime_index = a_index;
		if(b_prime_index == -1)
			b_prime_index = b_index;*/
		bool last = (b_index == (original_points.size() - 1) && a_index == 0);
		bool will_last = (a_index == (original_points.size() - 1) && b_index == 0);
		int diff_ap_b = difference_in_polygon(a_prime_index, b_index, original_points.size());
		int diff_ap_bp = difference_in_polygon(a_prime_index, b_prime_index, original_points.size());

		int diff_b_ap = difference_in_polygon(b_index, a_prime_index, original_points.size());
		int diff_b_a = difference_in_polygon(b_index, a_index, original_points.size());

		//if ((a_index > b_index && a_index > -1 && b_index > -1 && !will_last) || last		
		//	||(b_index == -1 && a_index > -1)) {

		if ((diff_ap_bp < diff_ap_b && a_index > -1) || (a_index > -1 && b_index == -1) || (b_index == b_prime_index && diff_b_a < diff_b_ap))
		{
			if (a_index == -1 && b_index > -1) {
				//don't flip!
			}
			else {
				//reverse everything!
				int temp_a = a_index, temp_p = a_prime_index;
				Point_2 temp_ap = a, temp_pp = a_prime;
				a = b, a_prime = b_prime, a_index = b_index, a_prime_index = b_prime_index;
				b = temp_ap, b_index = temp_a, b_prime = temp_pp, b_prime_index = temp_p;
				diff_ap_b = difference_in_polygon(a_prime_index, b_index, original_points.size());
				diff_ap_bp = difference_in_polygon(a_prime_index, b_prime_index, original_points.size());
			}
		}


		//OutputDebugString(std::to_string(a_index).c_str());
		//OutputDebugString("\n");
		//OutputDebugString(std::to_string(a_prime_index).c_str());
		//OutputDebugString("\n");
		//OutputDebugString(std::to_string(b_index).c_str());
		//OutputDebugString("\n");
		//OutputDebugString(std::to_string(b_prime_index).c_str());
		//OutputDebugString("\n");
		//OutputDebugString("\n");

		//OutputDebugString((std::string("a x: ") + std::to_string(CGAL::to_double(a.x()))).c_str());
		//OutputDebugString("\n");
		//OutputDebugString((std::string("a y: ") + std::to_string(CGAL::to_double(a.y()))).c_str());
		//OutputDebugString("\n");
		//OutputDebugString((std::string("ap x: ") + std::to_string(CGAL::to_double(a_prime.x()))).c_str());
		//OutputDebugString("\n");
		//OutputDebugString((std::string("ap y: ") + std::to_string(CGAL::to_double(a_prime.y()))).c_str());
		//OutputDebugString("\n");
		//OutputDebugString("\n");

		//OutputDebugString((std::string("b x: ") + std::to_string(CGAL::to_double(b.x()))).c_str());
		//OutputDebugString("\n");
		//OutputDebugString((std::string("b y: ") + std::to_string(CGAL::to_double(b.y()))).c_str());
		//OutputDebugString("\n");
		//OutputDebugString((std::string("bp x: ") + std::to_string(CGAL::to_double(b_prime.x()))).c_str());
		//OutputDebugString("\n");
		//OutputDebugString((std::string("bp y: ") + std::to_string(CGAL::to_double(b_prime.y()))).c_str());
		//OutputDebugString("\n");
		//OutputDebugString("\n");

		std::vector<std::vector<Point_2>> pgns = {}, p_right = {};
		std::vector<Point_2> p_a = {}, p_left = {}, p_b = {}, p_r = {};
		std::vector<int> p_r_i, p_a_i, p_l_i, p_b_i, p_right_i;


		if (a_index > -1)
			vertices_hit.insert(vertices_hit.begin(), a_index);
		if (b_index > -1)
			vertices_hit.push_back(b_index);

		if (a_index != a_prime_index) {
			if (a_index > -1) { //a is reflex
				p_a = { a,a };
				p_a_i = { a_index, a_index };
				p_a = add_points_between(p_a, original_points, a_prime_index, a_index);
			}
			else { //a is floating
				if (b_index == -1) {
					//both floating
					p_r = { b, a };
					p_r = add_points_between(p_r, original_points, a_prime_index, b_prime_index);
					p_r.push_back(b_prime);
					p_right.push_back(p_r);

					p_left = { a,b };
					p_left = add_points_between(p_left, original_points, b_prime_index, a_prime_index);
					p_left.push_back(a_prime);
				}
				else {
					//only a floating
					if (b_prime_index == b_index || diff_ap_bp > diff_ap_b) {
						p_r = { b, a };
						p_r = add_points_between(p_r, original_points, a_prime_index, b_index);
						p_r.push_back(b);
						p_right.push_back(p_r);

						p_left = { a, b };
						p_left = add_points_between(p_left, original_points, b_prime_index, a_prime_index);
						p_left.push_back(a_prime);
					}
					else {
						p_r = { b, a };
						p_r = add_points_between(p_r, original_points, a_prime_index, b_prime_index);
						p_r.push_back(b_prime);
						p_right.push_back(p_r);

						p_left = { a,b };
						p_left = add_points_between(p_left, original_points, b_index, a_prime_index);
						p_left.push_back(a_prime);
					}
				}
			}
		}

		if (b_index != b_prime_index && b_index > -1) { //b is reflex
			if (diff_ap_b > diff_ap_bp) {
				p_b = { b, b };
				p_b = add_points_between(p_b, original_points, b_prime_index, b_index);
			}
			else {
				p_b = { b };
				p_b = add_points_between(p_b, original_points, b_index, b_prime_index);
				p_b.push_back(b_prime);
			}
		}

		if (b_index > -1 && a_index > -1) {//neither floating:
			//p_r sorted by for loop
			int v_size = vertices_hit.size();
			for (int i = 0; i < (v_size - 1);i++) {
				int next = i + 1;
				p_r = { original_points[vertices_hit[next]], original_points[vertices_hit[i]] };
				p_r = add_points_between(p_r, original_points, vertices_hit[i], vertices_hit[next]);
				p_r.push_back(original_points[vertices_hit[next]]);
				if (p_r.size() > 4)
					p_right.push_back(p_r);
			}
			//p_left simple
			p_left = { a };
			if (b_prime_index != b_index)
				p_left.push_back(b);

			p_left = add_points_between(p_left, original_points, b_prime_index, a_prime_index);
			if (a_prime_index != a_index)
				p_left.push_back(a_prime);

			//if(p_right.size() < 1)
				//p_left.push_back(a);
		}

		pgns.push_back(p_b);
		pgns.insert(pgns.end(), p_right.begin(), p_right.end());
		pgns.push_back(p_left);
		pgns.push_back(p_a);
		return pgns;
	}


	std::vector<Point_2> compute_visibility_from_point(Point_2 a, Arrangement_2& env, TEV* tev)
	{
		//Initialize variables
		std::vector<Segment_2> segments;
		std::vector<Point_2> points = {};

		//Find the face
		Arrangement_2::Face_const_handle* face;
		CGAL::Arr_naive_point_location<Arrangement_2> pl(env);
		CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(a);

		// The query point locates in the interior of a face
		face = boost::get<Arrangement_2::Face_const_handle>(&obj);
		Arrangement_2 output_arr;
		typedef CGAL::Simple_polygon_visibility_2<Arrangement_2, CGAL::Tag_false> NSPV;
		Face_handle fh;
		Halfedge_const_handle he = Halfedge_const_handle();


		//If the point is within a face, we can compute the visbility that way
		if (face != NULL)
		{
			fh = tev->compute_visibility(a, *face, output_arr);
		}
		else
		{
			//If the point in a boundary segment, find the corresponding half edge
			he = env.halfedges_begin();
			bool cont = !Segment_2(he->source()->point(), he->target()->point()).has_on(a) || he->source()->point() == a || he->face()->is_unbounded();
			//While we are not in the right half edge, or while q is the source, continue
			while (cont) {
				he++;
				if (he == env.halfedges_end()) {
					throw(std::exception());
				}

				cont = !Segment_2(he->source()->point(), he->target()->point()).has_on(a) || he->source()->point() == a || he->face()->is_unbounded();
			}

			//Use the half edge to compute the visibility
			fh = tev->compute_visibility(a, he, output_arr);
		}
		//Make sure the visibility polygon we find has an outer boundary
		if (fh->has_outer_ccb()) {
			Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();

			//find the right halfedge first
			if (he != Halfedge_const_handle())
				while (++curr != fh->outer_ccb())
					if (curr->source()->point() == he->source()->point())
						break;

			Arrangement_2::Ccb_halfedge_circulator first = curr;
			points.push_back(curr->source()->point());

			//Save the points from the visibility polygon
			while (++curr != first)
				points.push_back(curr->source()->point());
		}

		return points;
	}

	Polygon_2 compute_visibility_from_segment(std::vector<Point_2>& poly, Point_2 a, Point_2 b) {
		//check if we need to split the orignal polygon into more polygons 
		std::vector<std::vector<Point_2>> split_pgns = split_for_vis(a, b, poly);


		std::vector<Point_2> points = {};

		if (same(poly[0], a) && same(poly[1], b)) {
			points.push_back(a);
			points.push_back(b);

		}

		//points.push_back(b);

		//compute visibility per split polygon and then combine
		for (int k = 0; k < split_pgns.size();k++)
		{
			//initiliaze variables
			int R = 0;
			int L = 0;
			std::vector<Point_2> current_pgn_points = split_pgns[k];
			if (current_pgn_points.size() < 3)
				continue;

			//This means we are computing visibility for a point and not segment
			if (same(current_pgn_points[0], current_pgn_points[1]))
			{
				//Erase the first point because it is duplicated
				current_pgn_points.erase(current_pgn_points.begin());

				if (current_pgn_points.size() < 3)
					continue;

				//compute the visibility using the method
				Arrangement_2 env = polygon_to_arrangement(current_pgn_points);
				TEV tev(env);
				std::vector<Point_2> vis_points =
					compute_visibility_from_point(current_pgn_points[0],
						env, &tev);

				//If the method results a valid polygon
				if (vis_points.size() > 0) {
					std::vector<Point_2> ordered_points = {};
					int viewer = -1;

					for (int i = 0; i < vis_points.size(); i++) {
						//Find index of viewer
						if (vis_points[i] == current_pgn_points[0]) {
							viewer = i;
							break;
						}
					}
					if (viewer == vis_points.size() - 1)
						points.insert(points.end(), vis_points.begin(), vis_points.end() - 1);
					else {
						points.insert(points.end(), vis_points.begin() + viewer + 1, vis_points.end());
						points.insert(points.end(), vis_points.begin(), vis_points.begin() + viewer);
					}
				}
				continue; //skip to the next iteration
			}

			//Loop through the vertices
			for (int i = 1; i < current_pgn_points.size() - 1; i++)
			{
				if (current_pgn_points[i + 1] == current_pgn_points[1]) {
					points.push_back(current_pgn_points[i + 1]);
					continue;
				}


				//Check for the righmost beam
				std::tuple<int, int> rmb = find_rightmost_beam(split_pgns[k], i);
				R = std::get<0>(rmb);
				L = std::get<1>(rmb);

				//No proper beam available!
				if (R == 0 && L == 0)
				{
					continue;
				}
				if (L == i + 1)
				{
					//add vi1 to result
					points.push_back(current_pgn_points[i + 1]);
					Point_2 s = empty_point;
					//point closest to v0 is a, so the source.				

					//compute point visibility
					Arrangement_2 env = polygon_to_arrangement(poly);
					TEV tev(env);
					std::vector<Point_2> point_vis = compute_visibility_from_point(current_pgn_points[i + 1], env, &tev);

					CGAL::Bounded_side b = CGAL::bounded_side_2(point_vis.begin(), point_vis.end(), current_pgn_points[0]);

					if (b != CGAL::ON_UNBOUNDED_SIDE) //a is visible
						s = current_pgn_points[0];
					else {
						//iterate the polygon and find the part of the edge that is inside
						for (int i_vis = 0; i_vis < point_vis.size();i_vis++) {
							Point_2 curp = point_vis[i_vis];
							Point_2 nextp = point_vis[(i_vis + 1) % point_vis.size()];

							Point_2 interpoint = point_intersection(Segment_2(current_pgn_points[0], current_pgn_points[1]),
								Segment_2(curp, nextp));
							if (isset(interpoint) && Segment_2(current_pgn_points[0], interpoint).squared_length() < Segment_2(current_pgn_points[0], s).squared_length())
								s = interpoint;
						}
					}

					//is s in LHP(vi1,vi2)?
					if (!isset(s) || current_pgn_points.size() <= (i + 2) || in_half_plane({ CGAL::LEFT_TURN,  CGAL::COLLINEAR }, current_pgn_points[i + 1], current_pgn_points[i + 2], s))
						continue;
					else
					{

						//x is the point where ray svi1 exits P, and vivi1 is the edge in which this happens (update i)
						std::tuple<Segment_2, int> sx_and_i = polygon_exits(Ray_2(s, current_pgn_points[i + 1]), current_pgn_points, current_pgn_points[i + 1], i + 1);
						Point_2 x = std::get<0>(sx_and_i).target();
						if (!same(s, x) && !same(current_pgn_points[i + 1], x))
						{

							points.push_back(x);
							i = std::get<1>(sx_and_i) - 1;
						}
					}
				}
				else
				{
					//Use R and L to find the left most point on vivi1 that we can see
					Point_2 q = point_intersection(Ray_2(current_pgn_points[R], current_pgn_points[L]), Segment_2(current_pgn_points[i], current_pgn_points[i + 1]));
					points.push_back(q);
					points.push_back(current_pgn_points[L]);

					//any i between i and L we cannot see, so i = L
					i = L - 1;
				}
			}
		}

		//remove duplicates
		std::vector<Point_2> pure_points = {};
		for (int i = 0; i < points.size();i++) {
			int next = (i + 1) % points.size();
			if (!same(points[i], points[next]))
				pure_points.push_back(points[i]);
		}


		return Polygon_2(pure_points.begin(), pure_points.end());
	}

};