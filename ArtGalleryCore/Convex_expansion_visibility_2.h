#ifndef CGAL_CONVEX_EXPANSION_VISIBILITY_2_H
#define CGAL_CONVEX_EXPANSION_VISIBILITY_2_H

#include <CGAL/license/Visibility_2.h>
#include <boost/shared_ptr.hpp>
#include <CGAL/boost/iterator/transform_iterator.hpp>
#include <CGAL/Arr_observer.h>
#include <CGAL/assertions.h>
#include <cassert>
#include <CGAL/use.h>
#include <vector>
#include <list>
#include <CGAL/Polygon_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

namespace CGAL {
    template<class Arrangement_2_, class RegularizationCategory = CGAL::Tag_true > 
    class Convex_expansion_visibility_2 {
        typedef typename Arrangement_2_::Geometry_traits_2    Geometry_traits_2;
        typedef typename Geometry_traits_2::Kernel            K;

        typedef Convex_expansion_visibility_2<
            Arrangement_2_, RegularizationCategory>             Self;

    public:
        typedef Arrangement_2_                                Arrangement_2;
        typedef typename Arrangement_2::Traits_2              Traits_2;
        typedef typename Arrangement_2::Halfedge              Halfedge;
        typedef typename Arrangement_2::Halfedge_const_handle Halfedge_const_handle;
        typedef typename Arrangement_2::Halfedge_handle       Halfedge_handle;
        typedef typename Arrangement_2::Edge_const_iterator   Edge_const_iterator;
        typedef typename Arrangement_2::Ccb_halfedge_const_circulator
            Ccb_halfedge_const_circulator;
        typedef typename Arrangement_2::Ccb_halfedge_circulator
            Ccb_halfedge_circulator;
        typedef typename Arrangement_2::Face_const_handle     Face_const_handle;
        typedef typename Arrangement_2::Face_handle           Face_handle;
        typedef typename Arrangement_2::Vertex_const_handle   Vertex_const_handle;
        typedef typename Arrangement_2::Vertex_handle         Vertex_handle;

        typedef typename K::Point_2                           Point_2;
        typedef typename Geometry_traits_2::Ray_2             Ray_2;
        typedef typename Geometry_traits_2::Segment_2         Segment_2;
        typedef typename Geometry_traits_2::Line_2            Line_2;
        typedef typename Geometry_traits_2::Vector_2          Vector_2;
        typedef typename Geometry_traits_2::Direction_2       Direction_2;
        typedef typename Geometry_traits_2::FT                Number_type;
        typedef typename Geometry_traits_2::Object_2          Object_2;

        typedef RegularizationCategory                       Regularization_category;

        typedef CGAL::Tag_true                      Supports_general_polygon_category;
        typedef CGAL::Tag_true                      Supports_simple_polygon_category;

    private:
       

        // Observer to track any changes of the attached arrangement.
        class Observer : public Arr_observer<Arrangement_2>
        {

            typedef Arr_observer<Arrangement_2>                           Base;
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


    private:
        const Arrangement_2* p_arr;

        // May change during visibility computation
        mutable Observer observer;
       // mutable boost::shared_ptr<CDT> p_cdt;
        mutable std::vector<Segment_2> needles;

        // Copy constructor and assignment not supported
        Convex_expansion_visibility_2(const Self&);
        Self& operator= (const Self&);


    public:
        Arrangement_2 convex_partition;

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
            if (p_arr != &arr) {
                p_arr = &arr;
                observer.detach();
                observer.attach(const_cast<Arrangement_2&>(arr));
                init_partition();
            }
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


        template <typename VARR>
        typename VARR::Face_handle
            compute_visibility(const std::vector<Point_2>& q,
                VARR& out_arr)
            const {
            if (observer.has_changed) {
                //recompute convex decomposition
                init_partition();
            }

            out_arr.clear();
            needles.clear();
           
            std::vector<Point_2> raw_output;
            

            return output(raw_output, out_arr);
        }

        
    private:     

        template <typename VARR>
        typename VARR::Face_handle
            output(std::vector<Point_2>& raw_output, VARR& out_arr) const {

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
                typename VARR::Vertex_handle v_last, v_first;
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

        void init_partition() {
            std::list<CGAL::Polygon_2<K>> partition_polys;
            std::vector<Point_2> vertices;

            //Construct vertex list
            auto eit = p_arr->unbounded_face()->inner_ccbs_begin();
            do {
                vertices.push_back((*eit)->source()->point());
            } while (++eit != p_arr->unbounded_face()->inner_ccbs_begin());

            //Create the partition
            CGAL::optimal_convex_partition_2(vertices.begin(),
                vertices.end(),
                std::back_inserter(partition_polys));

            //Verify the partition
            assert(CGAL::partition_is_valid_2(vertices.begin(),
                vertices.end(),
                partition_polys.begin(),
                partition_polys.end()));

            std::vector<Segment_2> segments;

            for (auto it = partition_polys.begin(); it != partition_polys.end(); it++) {
                for (auto eit = (*it).edges_begin(); eit != (*it).edges_end(); eit++) {
                    insert(convex_partition, Segment_2(eit->source(), eit->target()));
                }
            }
        }

        
    }; 

} // namespace CGAL

#endif // CGAL_TRIANGULAR_EXPANSION_VISIBILITY_2_H
