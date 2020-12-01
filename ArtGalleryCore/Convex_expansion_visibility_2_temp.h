#pragma once
#include "pch.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <cassert>
#include <list>

typedef CGAL::Direction_2<K> Direction_2;

class Convex_expansion_visibility_2 {
public:
    typedef Convex_expansion_visibility_2 Self;
    Arrangement_2 convex_partition;
    std::vector<Segment_2> saved_a_tangents;
    std::vector<Segment_2> saved_b_tangents;
    std::vector<Segment_2> saved_first_a_tangents;
    std::vector<Segment_2> saved_first_b_tangents;
    std::vector<std::vector<Point_2>> saved_alpha_blockers;
    std::vector<std::vector<Point_2>> saved_beta_blockers;

    std::vector<std::list<Point_2>> vis_polygons;
    std::vector<Point_2> f0_boundary;

private:
    std::vector<Halfedge_handle> diagonals;
    Trap_pl pointLocation;
    std::vector<Segment_2> q;
    std::vector<std::vector<Arrangement_2::Face_const_handle>> q_faces;


    class Observer : public CGAL::Arr_observer<Arrangement_2>
    {

        typedef CGAL::Arr_observer<Arrangement_2>                     Base;
        typedef Observer                                              Self;


    public:
        bool has_changed;

        Observer() : Base(), has_changed(false)
        {}

        Observer(const Arrangement_2& arr)
            : Base(const_cast<Arrangement_2&>(arr)), has_changed(false)
        {}

        // Arr_observer interface

        void after_attach() { has_changed = false; }


        void after_global_change() { has_changed = true; }
        void after_create_vertex(Vertex_handle) { has_changed = true; }
        void after_create_boundary_vertex(Vertex_handle) { has_changed = true; }
        void after_create_edge(Halfedge_handle) { has_changed = true; }
        void after_modify_vertex(Vertex_handle) { has_changed = true; }
        void after_modify_edge(Halfedge_handle) { has_changed = true; }
        void after_split_edge(Halfedge_handle, Halfedge_handle) {
            has_changed = true;
        }
        void after_split_fictitious_edge(Halfedge_handle, Halfedge_handle) {
            has_changed = true;
        }
        void after_split_face(Face_handle, Face_handle, bool) {
            has_changed = true;
        }
        void after_split_outer_ccb(Face_handle, Ccb_halfedge_circulator,
            Ccb_halfedge_circulator) {
            has_changed = true;
        }
        void after_split_inner_ccb(Face_handle, Ccb_halfedge_circulator,
            Ccb_halfedge_circulator) {
            has_changed = true;
        }
        void after_add_outer_ccb(Ccb_halfedge_circulator) { has_changed = true; }
        void after_add_inner_ccb(Ccb_halfedge_circulator) { has_changed = true; }
        void after_add_isolated_vertex(Vertex_handle) { has_changed = true; }
        void after_merge_edge(Halfedge_handle) { has_changed = true; }
        void after_merge_fictitious_edge(Halfedge_handle) { has_changed = true; }
        void after_merge_face(Face_handle) { has_changed = true; }
        void after_merge_outer_ccb(Face_handle, Ccb_halfedge_circulator) {
            has_changed = true;
        }
        void after_merge_inner_ccb(Face_handle, Ccb_halfedge_circulator) {
            has_changed = true;
        }
        void after_move_outer_ccb(Ccb_halfedge_circulator) { has_changed = true; }
        void after_move_inner_ccb(Ccb_halfedge_circulator) { has_changed = true; }
        void after_move_isolated_vertex(Vertex_handle) { has_changed = true; }
        void after_remove_vertex() { has_changed = true; }
        void after_remove_edge() { has_changed = true; }
        void after_remove_outer_ccb(Face_handle) { has_changed = true; }
        void after_remove_inner_ccb(Face_handle) { has_changed = true; }
    };

    const Arrangement_2* p_arr;

    // May change during visibility computation
    mutable Observer observer;
    // mutable boost::shared_ptr<CDT> p_cdt;
    mutable std::vector<Segment_2> needles;

    // Copy constructor and assignment not supported
    Convex_expansion_visibility_2(const Self&);
    Self& operator= (const Self&);

public:
    Convex_expansion_visibility_2() : p_arr(NULL) {}

    /*! Constructor given an arrangement. */
    Convex_expansion_visibility_2(const Arrangement_2& arr)
        : p_arr(&arr), observer(arr)
    {
        init_partition();
    }

    //const std::string name() const { return std::string("T_visibility_2"); }


    bool is_attached() const {
        return (p_arr != NULL);
    }

    void attach(const Arrangement_2& arr) {
        p_arr = &arr;
        observer.detach();
        observer.attach(const_cast<Arrangement_2&>(arr));
        init_partition();       
        //std::cout << "attach done" << std::endl;
    }

    void detach() {
        //std::cout << "detach" << std::endl;
        observer.detach();
        p_arr = NULL;
        convex_partition.clear();
    }

    const Arrangement_2& arrangement_2() const {
        return *p_arr;
    }


    Arrangement_2::Face_handle
        compute_visibility(const std::vector<Segment_2>& q,
            Arrangement_2& out_arr)
    {
        if (observer.has_changed) {
            //recompute convex decomposition
            init_partition();
        }
        //reset all drawing variables
        saved_a_tangents = saved_b_tangents = saved_first_a_tangents = saved_first_b_tangents = {};
        saved_alpha_blockers = saved_beta_blockers = {};
        vis_polygons = {};


        out_arr.clear();
        needles.clear();

        std::vector<Point_2> raw_output;   
        std::vector<Arrangement_2::Face_const_handle> c;
        q_faces = {};
        this->q = q;

        //Locate all face endpoints
        for (auto sit = q.begin(); sit != q.end(); sit++) {
            CGAL::Arr_point_location_result<Arrangement_2>::Type obj;

           // cout << CGAL::to_double(sit->source().x()) << " and " << CGAL::to_double(sit->source().y()) << "\n";

            obj = pointLocation.locate(sit->source());

            Arrangement_2::Face_const_handle loc_face;
            Arrangement_2::Vertex_const_handle loc_v;
            Arrangement_2::Halfedge_const_handle loc_he;

            std::vector<Arrangement_2::Face_const_handle> fhandles;

            if (CGAL::assign(loc_face, obj))
                fhandles = { loc_face }; //we are isolated in a face
            else if (CGAL::assign(loc_v, obj))
            {
                if (loc_v->is_isolated())
                    fhandles = { loc_v->face() };
                else {
                    auto he = loc_v->incident_halfedges();
                    do {
                        if (he->face()->has_outer_ccb())
                            fhandles.push_back(he->face());
                        else
                            fhandles.push_back(he->twin()->face());
                    } while (++he != loc_v->incident_halfedges());
                }
            }
            else if (CGAL::assign(loc_he, obj))
            {
                if (loc_he->face()->has_outer_ccb())
                    fhandles = { loc_he->face() };
                else
                    fhandles = { loc_he->twin()->face() };  
            }

            q_faces.push_back(fhandles);
            int crisb = fhandles.size();
            int cris2b = 22;
        }

        //find boundary of F0
        f0_boundary = {};

       find_F0(convex_partition.non_const_handle((*q_faces.begin()->begin())->outer_ccb()));

        for (auto eit = convex_partition.halfedges_begin(); eit != convex_partition.halfedges_end(); eit++)
        {
            eit->data().visited = false;
        }


        //We have located the first face start the recursive process of adding to the output from here
        points_from_visible_polygon(convex_partition.non_const_handle((*q_faces.begin()->begin())->outer_ccb()), raw_output, true);

        for (auto eit = convex_partition.halfedges_begin(); eit != convex_partition.halfedges_end(); eit++)
        {
            eit->data().visited = false;
        }

        //filter output for duplicates...
        for (auto pit = raw_output.begin(); pit != raw_output.end(); pit++) {
            auto next = ((pit + 1 == raw_output.end()) ? raw_output.begin() : pit + 1);
            auto next_next = ((next + 1 == raw_output.end()) ? raw_output.begin() : next + 1);


            if (*pit == *next) {
                raw_output.erase(next); //duplicate :(
                pit--;
            }
            //if (Segment_2(*pit, *next_next).has_on(*next)) {
            //    //there is no point in having "next"
            //    raw_output.erase(next);
            //    pit--;
            //}
        }


        return output(raw_output, out_arr);
    }

    
private:
    void find_F0(Arrangement_2::Halfedge_const_handle first)
    {
        auto eit = first;
        //iterate over face
        do {
            //been here already
            if (eit->data().visited)
                return;
            convex_partition.non_const_handle(eit)->data().visited = true;
            if (eit->data().is_diagonal)            {
                if (diagonal_intersects(eit->curve())) {
                    //check out this one too?
                    convex_partition.non_const_handle(eit)->twin()->data().visited = true;
                    find_F0(eit->twin()->next());
                }
                else   //eit is a diagonal, if we don't intersect its subpolygon is not part of F0
                    f0_boundary.push_back(eit->source()->point());
            }
            else {
                //this is part of the boundary?
                f0_boundary.push_back(eit->source()->point());
            }
            eit = eit->next();
        } while (eit != first);
    }

    void points_from_visible_polygon(
        Arrangement_2::Halfedge_const_handle first,
        std::vector<Point_2>& raw_output, bool first_time = false) {
        auto eit = first;
        //iterate over face
        do {
            //been here already
            if (eit->data().visited)
                return;

            convex_partition.non_const_handle(eit)->data().visited = true;

            if (!eit->data().is_diagonal) {
                //add the point, q sees it
                raw_output.push_back(eit->source()->point());
            }
            else {
                //eit is a diagonal
                convex_partition.non_const_handle(eit)->twin()->data().visited = true;

                //if q intersects the diagonal, q sees the whole polygon
                if (diagonal_intersects(eit->curve()))
                {
                    //we recursively add points in the next polygon (so the face of twin())
                    points_from_visible_polygon(eit->twin()->next(), raw_output);
                }
                else {
                    //we must expand      
                    Point_2 alpha = eit->twin()->target()->point();
                    Point_2 beta = eit->twin()->source()->point();

                    Point_2 alpha_tangent;
                    Point_2 beta_tangent;
                    std::vector<std::vector<Segment_2>::iterator> tangent_it = find_tangents(alpha_tangent, beta_tangent, alpha, beta, q.begin(), q.begin());
                                    
                    std::vector<Point_2> initial_alpha_blockers;
                    std::vector<Point_2> initial_beta_blockers;

                    //the beta blockers block the alpha tangent and the alpha blockers block the beta tangent

                    path_to_tangent(alpha_tangent, get_vertex_face(alpha_tangent), eit->next(), {}, initial_alpha_blockers, false);
                    path_to_tangent(beta_tangent, get_vertex_face(beta_tangent), eit->next(), {}, initial_beta_blockers, true);

                    std::vector<Point_2> initial_a_blocker = {};
                    std::vector<Point_2> initial_b_blocker = {};

                    if (!initial_alpha_blockers.empty())
                        initial_a_blocker = { best_blocker(alpha_tangent, initial_alpha_blockers, CGAL::LEFT_TURN) }; 
                    if (!initial_beta_blockers.empty())
                        initial_b_blocker = { best_blocker(beta_tangent, initial_beta_blockers, CGAL::RIGHT_TURN) };

                    int subs = expand_edge(eit->next(), eit->twin(), alpha, beta, initial_b_blocker, initial_a_blocker, alpha_tangent, beta_tangent, tangent_it, raw_output);
                }
            }
            eit = eit->next();
        } while (eit != first);
    }

    Point_2 best_blocker(Point_2 diagonal_vertex, std::vector<Point_2> candidates, CGAL::Orientation side) {
        auto rit = candidates.begin();
        Point_2 best_blocker = *rit;
        rit = rit + 1;
        while (rit != candidates.end()) {
            //is this a better blocker?
            if (CGAL::orientation(diagonal_vertex, *rit, best_blocker) == side)
                best_blocker = *rit;
            rit++;
        }

        return best_blocker;
    }

    int expand_edge(Arrangement_2::Halfedge_const_handle first_diagonal, Arrangement_2::Halfedge_const_handle diagonal,
        Point_2 a_prime, Point_2 b_prime, vector<Point_2> beta_blockers, vector<Point_2> alpha_blockers, Point_2 prev_a_tangent, Point_2 prev_b_tangent, std::vector<std::vector<Segment_2>::iterator> tangent_it,
        std::vector<Point_2>& raw_output) {

        int sub_count = 1; //ourself

        if (a_prime == b_prime)
            return 0; //this is a needle            

        //for drawing...
        Segment_2 alpha_t;
        Segment_2 beta_t;
        Segment_2 first_alpha_t;
        Segment_2 first_beta_t;
        list<Point_2> added_points;

        vector<Point_2> alpha_blockers_t;
        vector<Point_2> beta_blockers_t;

        Point_2 alpha_tangent;
        Point_2 beta_tangent;

        find_tangents(alpha_tangent, beta_tangent, a_prime, b_prime, tangent_it[0], tangent_it[1]);

        //check with previous tangent endpoints, do we need to add another blocker at the top? (is this really necessary?)
        if (alpha_tangent != prev_a_tangent) {
            std::vector<Point_2> new_initial_blockers;

            //update alpha_blockers (so we update "seen betas")
            path_to_tangent(alpha_tangent, get_vertex_face(alpha_tangent), first_diagonal, {}, new_initial_blockers, false);
            if (!new_initial_blockers.empty())
                alpha_blockers.push_back(best_blocker(alpha_tangent, new_initial_blockers, CGAL::LEFT_TURN));

        }
        if (beta_tangent != prev_b_tangent) {
            std::vector<Point_2> new_initial_blockers;

            //update beta_blockers (so we update "seen alphas")
            path_to_tangent(beta_tangent, get_vertex_face(beta_tangent), first_diagonal, {}, new_initial_blockers, true);
            if (!new_initial_blockers.empty())
                beta_blockers.push_back(best_blocker(beta_tangent, new_initial_blockers, CGAL::RIGHT_TURN));
        }

        prev_a_tangent = alpha_tangent;
        prev_b_tangent = beta_tangent;

        //Find the visibility line, using previous blockers
        vector<Point_2> blocker_candidates_a = alpha_blockers;
        vector<Point_2> blocker_candidates_b = beta_blockers;
        blocker_candidates_a.push_back(alpha_tangent);
        blocker_candidates_b.push_back(beta_tangent);

        Point_2 alpha_vis = best_blocker(a_prime, blocker_candidates_a, CGAL::RIGHT_TURN);
        Point_2 beta_vis = best_blocker(b_prime, blocker_candidates_b, CGAL::LEFT_TURN);
                   
        if (Line_2(alpha_vis, a_prime) == Line_2(beta_vis, b_prime)) {
            raw_output.push_back(a_prime);
            return sub_count; //we only see the diagonal, whats the point? we don't want duplicates!
        }


        //check if the tangent goes on the "wrong" side of a', which means we can see everything on the alpha side
        bool alpha_will_intersect = a_prime != diagonal->target()->point() || !(CGAL::orientation(alpha_vis, a_prime, diagonal->next()->target()->point()) != CGAL::RIGHT_TURN);
        //vice-versa for beta side
        bool beta_will_intersect = b_prime != diagonal->source()->point() || !(CGAL::orientation(beta_vis, b_prime, diagonal->prev()->source()->point()) != CGAL::LEFT_TURN);
       // bool beta_will_intersect_rem = beta_will_intersect;

        alpha_t = Segment_2(alpha_vis, a_prime);
        beta_t = Segment_2(beta_vis, b_prime);
        alpha_blockers_t = alpha_blockers;
        beta_blockers_t = beta_blockers;

        //update seen reflexes, for children
        if (a_prime == diagonal->target()->point())
            beta_blockers.push_back(a_prime);
        if (b_prime == diagonal->source()->point())
            alpha_blockers.push_back(b_prime);

        //get the next half-edge
        auto eit = diagonal->next();

        //we know a_prime will be seen
        if (a_prime == eit->source()->point() && alpha_will_intersect) {
            added_points.push_back(a_prime);
            raw_output.push_back(a_prime);
        }

        //if it is to the left, we will intersect, otherwise not
        do {
            //been here already
            if (eit->data().visited)
                return sub_count;

            convex_partition.non_const_handle(eit)->data().visited = true;
            bool a_intersected = false;
            Point_2 a_inter_point;

            //Check if we intersect alpha, because then from here on out we will start seeing stuff
            if (alpha_will_intersect) {
                auto inter = CGAL::intersection(Ray_2(alpha_vis, a_prime), Segment_2(eit->source()->point(), eit->target()->point()));
                if (inter) {
                    if (const Segment_2* s = boost::get<Segment_2>(&*inter)) {
                        int crisb = 15;
                    }
                    else {
                        a_inter_point = *boost::get<Point_2 >(&*inter);
                        a_intersected = a_inter_point != a_prime;

                        if (a_intersected) {
                            alpha_t = Segment_2(alpha_vis, a_inter_point);
                            alpha_will_intersect = false; //we found it
                        }
                    }
                }
            }

            //Check if we intersect beta, because then we will not see anything anymore
            bool b_intersected = false;
            Point_2 b_inter_point;
            if (beta_will_intersect) {
                auto inter = CGAL::intersection(Ray_2(beta_vis, b_prime), Segment_2(eit->source()->point(), eit->target()->point()));
                if (inter) {
                    if (const Segment_2* s = boost::get<Segment_2>(&*inter)) {
                        int crisb = 15;
                    }
                    else {
                        b_inter_point = *boost::get<Point_2 >(&*inter);
                        b_intersected = b_inter_point != b_prime;

                        if (b_intersected) {
                            beta_t = Segment_2(beta_vis, b_inter_point);
                            beta_will_intersect = false;
                        }
                    }
                }
            }

            if (!eit->data().is_diagonal) {
                //unless we still have to find the alpha intersection, we add the point to the output
                if (!alpha_will_intersect && !a_intersected) {
                    added_points.push_back(eit->source()->point());
                    raw_output.push_back(eit->source()->point());
                }
                if (a_intersected) {
                    //add intersection to output!
                    added_points.push_back(a_inter_point);
                    raw_output.push_back(a_inter_point);
                }
                if (b_intersected)
                {
                    added_points.push_back(b_inter_point);
                    raw_output.push_back(b_inter_point);
                }
                //otherwise we don't add any points, we still have to find alpha!
            }
            else {
                //eit is a diagonal
                convex_partition.non_const_handle(eit)->twin()->data().visited = true;

                if (a_intersected && b_intersected) {
                    //don't add to raw_output, the next call will do that already
                    added_points.push_back(a_inter_point);

                    //expand on the next diagonal...
                    sub_count += expand_edge(first_diagonal, eit->twin(), a_inter_point, b_inter_point, beta_blockers, alpha_blockers, prev_a_tangent, prev_b_tangent, tangent_it, raw_output);

                    //raw_output.push_back(b_inter_point);
                    //added_points.push_back(b_inter_point);

                }
                else if (a_intersected) {
                    //don't add to raw_output, the next call will do that already
                    added_points.push_back(a_inter_point);

                    //expand on the next diagonal...
                    sub_count += expand_edge(first_diagonal, eit->twin(), a_inter_point, eit->twin()->source()->point(), beta_blockers, alpha_blockers, prev_a_tangent, prev_b_tangent, tangent_it, raw_output);
                }
                else if (b_intersected) {
                    //don't add to raw_output, the next call will do that already
                    added_points.push_back(eit->source()->point());

                    //expand on the next diagonal...
                    sub_count += expand_edge(first_diagonal, eit->twin(), eit->twin()->target()->point(), b_inter_point, beta_blockers, alpha_blockers, prev_a_tangent, prev_b_tangent, tangent_it, raw_output);
                    //raw_output.push_back(b_inter_point);
                    //added_points.push_back(b_inter_point);
                }
                else if (!alpha_will_intersect && !a_intersected) {
                    //this is a normal diagonal and we can see it wholly, expand on it

                     //don't add to raw_output, the next call will do that already
                    added_points.push_back(eit->source()->point());
                    sub_count += expand_edge(first_diagonal, eit->twin(), eit->twin()->target()->point(), eit->twin()->source()->point(), beta_blockers, alpha_blockers, prev_a_tangent, prev_b_tangent, tangent_it, raw_output);
                }
                else {
                    //we actually don't see this diagonal, because we still have to find a_tangent's intersection
                }
            }

            if (b_intersected)
                break;

            eit = eit->next();
        } while (eit != diagonal);


        //don't add to raw_output, the function calling us will add b_prime to the output
        added_points.push_back(b_prime);
        vis_polygons.push_back(added_points);
        saved_a_tangents.push_back(alpha_t);
        saved_b_tangents.push_back(beta_t);

        saved_alpha_blockers.push_back(alpha_blockers_t);
        saved_beta_blockers.push_back(beta_blockers_t);


        saved_first_a_tangents.push_back(first_alpha_t);
        saved_first_b_tangents.push_back(first_beta_t);

        return sub_count;
    }

    bool path_to_tangent(Point_2 tangent_point,
        std::vector<Arrangement_2::Face_const_handle> tangent_faces, Arrangement_2::Halfedge_const_handle first, std::vector<Point_2> path_so_far, std::vector<Point_2>& path, bool source) {
        auto eit = first;

        //in any of the tangent faces, we want the path to be as short as possible!
        for (auto tfit = tangent_faces.begin(); tfit != tangent_faces.end(); tfit++) {
            if (first->face() == *tfit) {
                path = path_so_far;
                //we are it
                return true;
            }
        }

        //iterate over face
        do {
            if (eit->data().is_diagonal) {
          
                if (diagonal_intersects(eit->curve())) {
                    std::vector<Point_2> child_path_so_far = path_so_far;
                    if(source)
                        child_path_so_far.push_back(convex_partition.non_const_handle(eit->twin())->source()->point());
                    else
                        child_path_so_far.push_back(convex_partition.non_const_handle(eit->twin())->target()->point());

                    //recursively check 
                    if (path_to_tangent(tangent_point, tangent_faces, eit->twin()->next(), child_path_so_far, path, source)) {
                        return true;
                    }
                }
            }
            eit = eit->next();
        } while (eit != first->prev());

        return false;
    }

    std::vector<Arrangement_2::Face_const_handle> get_vertex_face(Point_2 vertex) {
        auto qfit = q_faces.begin();
        for (auto sit = q.begin(); sit != q.end(); sit++) {
            if (sit->source() == vertex) {
                return *qfit;
            }
            qfit++;
        }
    }

    bool is_tangent(Point_2 p, Point_2 q, Point_2 prev, Point_2 next, CGAL::Orientation side) {
        CGAL::Orientation previous_orientation = CGAL::orientation(p, q, prev);
        CGAL::Orientation next_orientation = CGAL::orientation(p, q, next);

        //Collinear means always a tangent since our faces are convex. No general position!
        if ((next_orientation == CGAL::COLLINEAR && previous_orientation == side) || (previous_orientation == CGAL::COLLINEAR && next_orientation == side))
            return true;

        return (previous_orientation == next_orientation && next_orientation == side);
    }

    vector<std::vector<Segment_2>::iterator> find_tangents(Point_2 &alpha_tangent, Point_2 &beta_tangent, Point_2 a_prime, Point_2 b_prime, std::vector<Segment_2>::iterator prev_at, std::vector<Segment_2>::iterator prev_bt) {
        vector<std::vector<Segment_2>::iterator > tangent_it;

        
        if (q.size() == 2) {
            //we are a segment
            auto sit = q.begin();
            CGAL::Orientation a_target_orientation = CGAL::orientation(a_prime, sit->target(), sit->source());
            CGAL::Orientation b_target_orientation = CGAL::orientation(b_prime, sit->target(), sit->source());
            tangent_it = { q.begin(), q.begin() }; //not useful now.
            if (a_target_orientation == CGAL::LEFT_TURN) {
                alpha_tangent = sit->target();
            }
            else
            {
                alpha_tangent = sit->source();
            }

            if (b_target_orientation == CGAL::RIGHT_TURN) {
                beta_tangent = sit->target();
            }
            else
            {
                beta_tangent = sit->source();
            }
        }
        else {
            

            auto sit = prev_at;

            //Find tangent between face and diagonal
            do {
                Point_2 prev = sit->source();
                Point_2 next = ((sit + 1 == q.end()) ? q.begin() : sit + 1)->target();

                //the alpha tangent has the face to its left
                if (is_tangent(a_prime, sit->target(), prev, next, CGAL::LEFT_TURN)) {
                    alpha_tangent = sit->target();
                    tangent_it.push_back(sit);
                    break;
                }

                //update sit (we are going CCW now)
                sit = (sit + 1 == q.end() ? q.begin() : sit + 1);
            } while (sit != prev_at);
            sit = prev_bt;


            do {
                Point_2 prev = sit->source();
                Point_2 next = ((sit + 1 == q.end()) ? q.begin() : sit + 1)->target();

                //the beta tangent has the face to its right
                if (is_tangent(b_prime, sit->target(), prev, next, CGAL::RIGHT_TURN)) {
                    beta_tangent = sit->target();
                    tangent_it.push_back(sit);
                    break;
                }

                //update sit (we are going CW now)
                sit = (sit == q.begin() ? q.end() - 1 : sit - 1);
            } while (sit != prev_bt);
        }

        return tangent_it;


    }
           
    bool diagonal_intersects(Segment_2 diagonal) {
        //check for any intersections      
        for (auto sit = q.begin(); sit != q.end();sit++) {
            if(CGAL::do_intersect(*sit, diagonal))
                return true;
        }
        return false;
    }
    
    Arrangement_2::Face_handle
        output(std::vector<Point_2>& raw_output, Arrangement_2& out_arr) const {

        if (!needles.empty()) {
            std::vector<Segment_2> segments(needles.begin(), needles.end());
            for (unsigned int i = 0; i < raw_output.size(); i++) {
                //      //std::cout <<  raw_output[i] << " -- " 
                //                <<  raw_output[(i+1)%raw_output.size()] << std::endl; 
                segments.push_back(Segment_2(raw_output[i],
                    raw_output[(i + 1) % raw_output.size()]));
            }

            CGAL::insert_non_intersecting_curves(out_arr,
                segments.begin(),
                segments.end());

        }
        else {
            typename Arrangement_2::Vertex_handle v_last, v_first;
            v_last = v_first =
                out_arr.insert_in_face_interior(raw_output[0], out_arr.unbounded_face());

            for (unsigned int i = 0; i < raw_output.size() - 1; i++) {
                //      std::cout <<  raw_output[i] << " -- "
                //                <<  raw_output[(i+1)%raw_output.size()] << std::endl;
                if (raw_output[i] < raw_output[(i + 1)]) {
                    v_last = out_arr.insert_from_left_vertex(
                        Segment_2(raw_output[i], raw_output[i + 1]), v_last
                    )->target();
                }
                else {
                    v_last = out_arr.insert_from_right_vertex(
                        Segment_2(raw_output[i], raw_output[i + 1]), v_last
                    )->target();
                }
            }
            out_arr.insert_at_vertices(
                Segment_2(raw_output.front(), raw_output.back()),
                v_last, v_first
            );
        }

        CGAL_assertion(out_arr.number_of_faces() == 2);

        if (out_arr.faces_begin()->is_unbounded())
            return ++out_arr.faces_begin();
        else
            return out_arr.faces_begin();
    }


    void init_partition()  {
        std::list<Polygon_2> partition_polys;
        std::vector<Point_2> vertices;

        //Construct vertex list
        auto eit = *p_arr->unbounded_face()->inner_ccbs_begin();
        do {            
            vertices.push_back(eit->source()->point());
        } while (++eit != *p_arr->unbounded_face()->inner_ccbs_begin());

        //Make counter clockwise
        std::reverse(vertices.begin(), vertices.end()); 

        //Create the partition
        CGAL::greene_approx_convex_partition_2(vertices.begin(),
            vertices.end(),
            std::back_inserter(partition_polys));

        //Polygon_with_holes_2 unionR;
      //  CGAL::join(partition_polys.begin(), partition_polys.end(), unionR);

              
        for (auto it = partition_polys.begin(); it != partition_polys.end(); it++) {
            for (auto eit = (*it).edges_begin(); eit != (*it).edges_end(); eit++) {
               CGAL::insert(convex_partition, Segment_2(eit->source(), eit->target()));
            }
        }




        //assign IDs
        auto ceit = *convex_partition.unbounded_face()->inner_ccbs_begin();
        int id = 0;
        do {
            ceit->source()->data().id = id;
            id++;
        } while (++ceit != *convex_partition.unbounded_face()->inner_ccbs_begin());

        //identify diagonals
        for (auto fit = convex_partition.faces_begin(); fit != convex_partition.faces_end(); fit++) {
            if (fit->has_outer_ccb()) {
                auto eit = fit->outer_ccb();
                do {
                    auto twin = eit->twin();
                    if (!twin->data().is_diagonal && twin->face()->has_outer_ccb())
                    {
                        //twin and us are diagonals
                        //we only have to save one of them though
                        twin->data().is_diagonal = true;
                        eit->data().is_diagonal = true;
                        diagonals.push_back(eit);
                    }
                } while (++eit != fit->outer_ccb());                

            }

        }
        pointLocation.attach(convex_partition);



    }
};