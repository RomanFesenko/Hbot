
#ifndef  _vecalg_
#define  _vecalg_
#include<math.h>

const double Mpi=3.14159265358979;
const double Degree=0.017453292519943;
// Требования к типу array_t: наличие оператора [int]
// и конструктор по координатам

template<class array_t>
auto distance_2D(const array_t& pt)->decltype(pt[0]+0)
{
    return  sqrt((pt[0])*(pt[0])+(pt[1])*(pt[1]));
}

template<class array_t>
array_t normalize_2D(const array_t& pt)
{
    return pt/distance_2D(pt);
}

// Повороты по и против часовой стрелке
template<class array_t>
array_t turn_clockwise(const array_t& pt)
{
    return array_t(pt[1],-pt[0]);
}

template<class array_t>
array_t turn_versus_clockwise(const array_t& pt)
{
    return array_t( -pt[1],pt[0]);
}

template<class array_t>
auto dot_product_2D(const array_t& pt1,const array_t& pt2)
->decltype(pt1[0]+0)
{
    return pt1[0]*pt2[0]+pt1[1]*pt2[1];
}

template<class array_t>
auto cross_product_2D(const array_t& pt1,const array_t& pt2)
->decltype(pt1[0]+0)
{
    return pt1[0]*pt2[1]-pt1[1]*pt2[0];
}

template<class array_t>
auto cos_angle_2D(const array_t& pt1,const array_t& pt2)
->decltype(pt1[0]+0)
{
    return dot_product_2D(pt1,pt2)/(distance_2D(pt1)*distance_2D(pt2));
}

// может быть меньше 0

template<class array_t>
auto sin_angle_2D(const array_t& pt1,const array_t& pt2)
->decltype(pt1[0]+0)
{
    return cross_product_2D(pt1,pt2)/(distance_2D(pt1)*distance_2D(pt2));
}


template<class array_t>
auto cos_sin_angle_2D(const array_t& pt1,const array_t& pt2)
->std::pair<decltype(pt1[0]+0),decltype(pt1[0]+0)>
{
    decltype(pt1[0]+0) temp=distance_2D(pt1)*distance_2D(pt2);
    return std::make_pair(dot_product_2D(pt1,pt2)/temp,
                          cross_product_2D(pt1,pt2)/temp);
}

//angle_2D(array_t,array_t) -угол между векторами в интервале (-pi,+pi)
//отсчитываемый от 1 го вектора против часовой стрелке

template<class array_t>
auto angle_2D(const array_t& pt1,const array_t& pt2)
->decltype(pt1[0]+0)
{
    auto cos_sin=cos_sin_angle_2D(pt1,pt2);
    decltype(pt1[0]+0) res=acos(cos_sin.first);
    return (cos_sin.second>=0)? res:(-res);
}

//angle_2D(array_t) - угол между вектором и осью абцисс
// в интервале (-pi,+pi)

template<class array_t>
auto angle_2D(const array_t& pt1)
->decltype(pt1[0]+0)
{
    decltype(pt1[0]+0) res=acos(pt1[0]/distance_2D(pt1));
    return (pt1[1]>=0)? res:(-res);
}

template<class _Val>
void normalize_angle(_Val& vl)
{
    _Val _2_Mpi(2.0*Mpi);
    while(vl>Mpi){vl-=_2_Mpi;}
    while(vl<-Mpi){vl+=_2_Mpi;}
}


//////////////////////////////////////
// Векторная алгебра в пространстве///
//////////////////////////////////////

template<class array_t>
auto distance_3D(const array_t& pt)
->decltype(pt[2]+0)
{
    return  sqrt((pt[0])*(pt[0])+(pt[1])*(pt[1])+(pt[2])*(pt[2]));
}

template<class array_t>
array_t normalize_3D(const array_t& pt)
{
    decltype(pt[0]+0) dist=distance_3D(pt);
    return array_t(pt[0]/dist,pt[1]/dist,pt[2]/dist);
}

template<class array_t>
auto dot_product_3D(const array_t& pt1,const array_t& pt2)
->decltype(pt1[2]+0)
{
    return pt1[0]*pt2[0]+pt1[1]*pt2[1]+pt1[2]*pt2[2];
}

template<class array_t>
array_t cross_product_3D(const array_t& pt1,const array_t& pt2)
{
    return array_t( pt1[1]*pt2[2]-pt1[2]*pt2[1],
                    pt1[2]*pt2[0]-pt1[0]*pt2[2],
                    pt1[0]*pt2[1]-pt1[1]*pt2[0]);
}

template<class array_t>
auto cos_angle_3D(const array_t& pt1,const array_t& pt2)
->decltype(pt1[2]+0)
{
    return dot_product_3D(pt1,pt2)/(distance_3D(pt1)*distance_3D(pt2));
}


template <class T>
auto to_radian(const T& degrees)
{
    if constexpr(std::is_arithmetic<T>::value)
    {
        return Degree*static_cast<double>(degrees);
    }
    else
    {
        return degrees*Degree;
    }
}

template <class T>
auto to_degree(const T& rads)
{
    if constexpr(std::is_arithmetic<T>::value)
    {
        return static_cast<double>(rads)/Degree;
    }
    else
    {
        return rads/Degree;
    }
}


#endif

