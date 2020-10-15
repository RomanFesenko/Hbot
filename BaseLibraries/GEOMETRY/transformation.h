
#ifndef  _transformation_
#define  _transformation_
#include "../LINALG/linalg.h"
#include "vecalg.h"

//////////////////////////////////////////////////
//    ѕреобразовани€ на плоскости              ///
//////////////////////////////////////////////////


template<class real_t>
struct TransformMatrix2D:public FixedMatrix<real_t,3,3>{};


template<class array_t,class _Val>
array_t Transform(const TransformMatrix2D<_Val>& sm,const array_t&pt)
{
    return array_t
    (
        sm[0][0]*pt[0]+sm[0][1]*pt[1]+sm[0][2],
        sm[1][0]*pt[0]+sm[1][1]*pt[1]+sm[1][2]
    );
}

// матрица переноса

template<class array_t>
auto TranslateMx2D(const array_t& delta)
->TransformMatrix2D<decltype(delta[0]+0)>
{
    TransformMatrix2D<decltype(delta[0]+0)> res;
    res[0][2]=delta[0];
    res[1][2]=delta[1];
    return res;
}

// матрица масштабировани€ по данной неподвижной точке
// и масштабным коэффицентам

template<class array_t,class _Val>
TransformMatrix2D<_Val>
ScaleMx2D(const array_t& immove,_Val _scx,_Val _scy)
{
    TransformMatrix2D<_Val> res;
    res[0][0]=_scx;res[0][2]=(1.0-_scx)*immove[0];
    res[1][1]=_scy;res[1][2]=(1.0-_scy)*immove[1];
    return res;
}

// получение матрицы масштабировани€ по 2 парам точек
// преобразующимс€ друг в друга
template<class array_t>
auto ScaleMx2D(const array_t&from1,
          const array_t&from2,
          const array_t&to1,
          const array_t&to2)
->TransformMatrix2D<decltype(from1[0]+0)>
{
    using real_t=decltype(from1[0]+0);
    real_t x12=from1[0]-from2[0];
    real_t y12=from1[1]-from2[1];
    TransformMatrix2D<real_t> mx;
    mx[0][0]=(to1[0]-to2[0])/x12;
    mx[0][2]=(from1[0]*to2[0]-from2[0]*to1[0])/x12;

    mx[1][1]=(to1[1]-to2[1])/y12;
    mx[1][2]=(from1[1]*to2[1]-from2[1]*to1[1])/y12;
    return mx;
}


// ѕолучение матрицы поворoта вокруг неподвижной точки на данный угол
template<class array_t,class _Val>
TransformMatrix2D<_Val>
TurnMx2D(const array_t& immove,_Val angle)
{
    TransformMatrix2D<_Val> mx;
    _Val cang=cos(angle); _Val sang=sin(angle);
    mx[0][0]=cang; mx[0][1]=-sang;
    mx[1][0]=sang; mx[1][1]=cang;
    mx[0][2]=immove[0]*(1-cang)+immove[1]*sang;
    mx[1][2]=immove[1]*(1-cang)-immove[0]*sang;
    return mx;
}


// ѕолучение матрицы отражени€ относительно пр€мой проход€щей
// через данную точку под данным углом
template<class array_t,class _Val>
TransformMatrix2D<_Val>
ReflectMx2D(const array_t& line_point,_Val angle)
{
    TransformMatrix2D<_Val> mx;
    _Val cang=cos(2.0*angle); _Val sang=sin(2.0*angle);
    mx[0][0]=cang; mx[0][1]=sang;
    mx[1][0]=sang; mx[1][1]=-cang;
    mx[0][2]=line_point[0]*(1-cang)-line_point[1]*sang;
    mx[1][2]=line_point[1]*(1+cang)-line_point[0]*sang;
    return mx;
}

//////////////////////////////////////////////////
//    ѕреобразовани€ в пространстве            ///
//////////////////////////////////////////////////

template<class real_t>
struct TransformMatrix3D:public FixedMatrix<real_t,4,4>{};


template<class array_t,class _Val>
array_t Transform(const TransformMatrix3D<_Val>& sm,const array_t&pt)
{
    return array_t
    (
        sm[0][0]*pt[0]+sm[0][1]*pt[1]+sm[0][2]*pt[2]+sm[0][3],
        sm[1][0]*pt[0]+sm[1][1]*pt[1]+sm[1][2]*pt[2]+sm[1][3],
        sm[2][0]*pt[0]+sm[2][1]*pt[1]+sm[2][2]*pt[2]+sm[2][3]
    );
}

// матрица переноса

template<class array_t>
auto TranslateMx3D(const array_t& delta)
->TransformMatrix3D< decltype(delta[0]+0) >
{
    TransformMatrix3D<decltype(delta[0]+0)> res;
    res[0][3]=delta[0];
    res[1][3]=delta[1];
    res[2][3]=delta[2];
    return res;
}

// ѕолучение матрицы поворoта вокруг данной оси проход€щей
// через данную точку на данный угол

template<class array_t,class _Val>
TransformMatrix3D<_Val>
TurnMx3D(const array_t& ax,_Val angle)
{
    TransformMatrix3D<_Val> mx;
    _Val cang=cos(angle); _Val sang=sin(angle);
    array_t nax=normalize_3D(ax);
    mx[0][0]=cang+nax[0]*nax[0]*(1-cang);
    mx[0][1]=-nax[2]*sang+nax[0]*nax[1]*(1-cang);
    mx[0][2]=nax[1]*sang+nax[0]*nax[2]*(1-cang);

    mx[1][0]=mx[0][1]+2*nax[2]*sang;
    mx[1][1]=cang+nax[1]*nax[1]*(1-cang);
    mx[1][2]=-nax[0]*sang+nax[1]*nax[2]*(1-cang);

    mx[2][0]=mx[0][2]-2*nax[1]*sang;
    mx[2][1]=mx[1][2]+2*nax[0]*sang;
    mx[2][2]=cang+nax[2]*nax[2]*(1-cang);

    return mx;
}

template<class array_t,class _Val>
TransformMatrix3D<_Val>
TurnMx3D(const array_t& ax,_Val angle,const array_t& immove)
{
    auto mx=TurnMx3D(ax,angle);
    array_t nax=normalize_3D(ax);
    array_t temp=cross_product_3D(immove,nax);
    array_t temp2=cross_product_3D(nax,temp);
    _Val v1=1-cos(angle); _Val v2=sin(angle);
    mx[0][3]=v1*temp2[0]+v2*temp[0];
    mx[1][3]=v1*temp2[1]+v2*temp[1];
    mx[2][3]=v1*temp2[2]+v2*temp[2];
    return mx;
}

// ѕолучение матрицы отражени€ относительно плоскости проход€щей
// через данную точку с данной нормалью
template<class array_t>
auto ReflectMx3D(const array_t& norm_vec)
->TransformMatrix3D<decltype(norm_vec[0]+0)>
{
    TransformMatrix3D<decltype(norm_vec[0]+0)> mx;
    array_t nvec=normalize_3D(norm_vec);
    mx[0][0]=1-2*nvec[0]*nvec[0];
    mx[0][1]=-2*nvec[0]*nvec[1];
    mx[0][2]=-2*nvec[0]*nvec[2];

    mx[1][0]=mx[0][1];
    mx[1][1]=1-2*nvec[1]*nvec[1];
    mx[1][2]=-2*nvec[1]*nvec[2];

    mx[2][0]=mx[0][2];
    mx[2][1]=mx[1][2];
    mx[2][2]=1-2*nvec[2]*nvec[2];
    return mx;
}

template<class array_t>
auto ReflectMx3D(const array_t& norm_vec,const array_t&immove)
->TransformMatrix3D<decltype(norm_vec[0]+0) >
{
    using value_type=decltype(norm_vec[0]+0);
    TransformMatrix3D<value_type> mx=ReflectMx3D(norm_vec);
    array_t nvec=normalize_3D(norm_vec);
    value_type dpr=2*dot_product_3D(nvec,immove);
    mx[0][3]=dpr*nvec[0]; mx[1][3]=dpr*nvec[1]; mx[2][3]=dpr*nvec[2];
    return mx;
}
#endif


///////////TEST ///////////////////

/*
double test_3D_turn_matrix()
{
    struct point{
        double _d[3];
        double& operator[](int i){return _d[i];}
        double operator[](int i)const{return _d[i];}
        point(){}
        point(double x,double y,double z)
        {_d[0]=x;_d[1]=y;_d[2]=z;}
    };
    point ax; point immove;
    ax[0]=2; ax[1]=3; ax[2]=4;
    immove[0]=5; immove[1]=6; immove[2]=7;
    auto mx=TurnMx3D(ax,Mpi/8,immove);
    auto mx_test=TurnMx3D(ax,-Mpi/4,immove);
    TransformMatrix3D<double> mx2,mx3;
    multiply_matrix(mx,mx,mx2);
    invert_matrix(mx2,mx3);//mx3==mx_test
    double res=0;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            res=std::max(res,abs(mx3[i][j]-mx_test[i][j]));
        }
    }
    return res;
}

void equiside_poligon()
{
    struct point{
        double _d[2];
        double& operator[](int i){return _d[i];}
        double operator[](int i)const{return _d[i];}
        point(){}
        point(double x,double y)
        {_d[0]=x;_d[1]=y;}
    };
    double ang=Mpi/7;
    auto mx=ReflectMx2D(point(0,0),ang);
}

*/


