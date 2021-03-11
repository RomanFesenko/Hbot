
#ifndef  _primitives_3D_
#define  _primitives_3D_

#include <math.h>
#include <array>
#include <tuple>
#include <algorithm>
#include <type_traits>
#include <assert.h>

#include "vecalg.h"

//////////////////////////////////////////////////////////////
//Beam in 3D
//////////////////////////////////////////////////////////////

template<class point_t>
class ort_3D_t
{
    point_t m_ort;
    public:
    explicit ort_3D_t(const point_t& vec):m_ort(normalize_3D(vec)){}
    point_t vector()const{return m_ort;}
};

template<class point_t>
struct line_3D_t // directed
{
    point_t base_point;
    using value_type=decltype(base_point[0]+0);
    ort_3D_t<point_t> ort;// (-pi...pi)
    line_3D_t(const point_t& pt,const ort_3D_t<point_t>& _ort):
    base_point(pt),ort(_ort)
    {

    }
    line_3D_t(const point_t& pt1,const point_t& pt2)// from pt1 to pt2
    :base_point(pt1),ort(ort_3D_t(pt2-pt1))
    {

    }
    void set(const point_t& pt1,const point_t& pt2)
    {
        base_point=pt1;
        ort=ort_3D_t(pt2-pt1);
    }
    point_t direct()const{return ort.vector();}
};


// determine the distance from a point to a straight line
// res [0] - actual distance
// res [1] - distance from line.base_point to point projection on a straight line
// along the positive direction of the straight line
template<class point_t>
point_t distance_3D(const line_3D_t<point_t>& line,const point_t& pt)
{
    using value_type=decltype(pt[0]+0);
    point_t dist=pt-line.base_point;
    value_type project=dot_product_3D(dist,line.direct());
    value_type sq_dist=dot_product_3D(dist,dist);
    return point_t(sqrt(sq_dist-project*project),project,0);
}

// the same but in addition the projection of the point is returned
template<class point_t>
point_t distance_3D(const line_3D_t<point_t>& line,const point_t& pt,
                                                    point_t&project_point)
{
    using value_type=decltype(pt[0]+0);
    point_t dist=pt-line.base_point;
    value_type project=dot_product_3D(dist,line.direct());
    value_type sq_dist=dot_product_3D(dist,dist);
    project_point=line.base_point+project*line.direct();
    return point_t(sqrt(sq_dist-project*project),project,0);
}

// rotate the vector in space

// counterclockwise around the axis
template<class point_t>
point_t turn_around_axis(const point_t& source,const point_t& axis,
                         decltype(source[0]+0) angle)
{
    using value_type=decltype(source[0]+0);
    point_t norm_axis=normalize_3D(axis);
    value_type cos_ang=cos(angle);
    value_type sin_ang=sin(angle);
    return source*cos_ang+cross_product_3D(norm_axis,source)*sin_ang+
           (1-cos_ang)*dot_product_3D(norm_axis,source)*norm_axis;
}

// in plane in the direction of another vector
template<class point_t>
point_t turn_in_plane(const point_t& source,const point_t& in_plane,
                      decltype(source[0]+0) angle)
{
    return turn_around_axis(source,cross_product_3D(source,in_plane),angle);
}

template<class point_t,class iterator_t>
std::array<point_t,2> close_box(iterator_t begin,iterator_t end)
{
    auto xcoord_comparer=[](const point_t&pt1,const point_t&pt2){return pt1[0]<pt2[0];};
    auto ycoord_comparer=[](const point_t&pt1,const point_t&pt2){return pt1[1]<pt2[1];};
    auto zcoord_comparer=[](const point_t&pt1,const point_t&pt2){return pt1[2]<pt2[2];};

    auto x_range=std::minmax_element(begin,end,xcoord_comparer);
    auto y_range=std::minmax_element(begin,end,ycoord_comparer);
    auto z_range=std::minmax_element(begin,end,zcoord_comparer);

    return {
    point_t((*(x_range.first))[0], (*(y_range.first))[1],(*(z_range.first))[2]),
    point_t((*(x_range.second))[0],(*(y_range.second))[1],(*(z_range.second))[2])
    };
}



#endif









