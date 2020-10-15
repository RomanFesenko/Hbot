
#ifndef  _algo_extend_
#define  _algo_extend_

#include <functional>
#include <algorithm>


// STL-подобные алгоритмы

// минимальные и максимальные элементы
// если функция сравнения принимает один аргумент

namespace ext
{

/*
template<class Func_t,class Arg1,class Arg2>
class is_binary_function
{
    template<class F,class I1,class I2>
    static double test(...){return 0;}// не является

    template<class F,class I1,class I2,class CHECK=
    decltype(std::declval<F>()( (std::declval<I1>()),
    (std::declval<I2>()) )) >
    static int test(void*);
    public:
    static constexpr bool value=std::is_same<int,
    decltype( test<Func_t,Arg1,Arg2>(nullptr) )>::value;
};

//

template<class Func_t,class Arg_t>
class is_unary_function
{
    template<class F,class I>
    static double test(...){return 0;} // не является

    template<class F,class I,class CHECK=
    decltype(std::declval<F>()( (std::declval<I>()) ) ) >
    static int test(void*);
    public:
    static constexpr bool value=std::is_same<int,
    decltype(test<Func_t,Arg_t>(nullptr))>::value;
};

// проверка на наличие в классе открытого оператора() (на функтор)
// лямбды тоже имеют такой член

template<class Func_t,class Arg_t>
class is_unary_functor
{
    template<class F,class I>
    static double test(...){return 0;} // не является

    template<class F,class I,class CHECK=
    decltype(std::declval<F>().operator()( std::declval<I>() ) ) >
    static int test(void*);
    public:
    static constexpr bool value=std::is_same<int,
    decltype(test<Func_t,Arg_t>(nullptr))>::value;
};

// проверка является ли тип указателем на функцию
// принимающую один аргумент типа Arg_t

template<class Func_t,class Arg_t>
class is_unary_function_ptr
{
    template<class F,class I>
    static double test(...){return 0;} // не является

    template<class F,class I,class CHECK=
    decltype(std::declval<F>()( std::declval<I>() ) ),// проверка на вызываемость
    class CHECK2=decltype( ((void*)std::declval<F>()) ) >// проверка на указательность
    static int test(void*);
    public:
    static constexpr bool value=std::is_same<int,
    decltype(test<Func_t,Arg_t>(nullptr))>::value;
};

template<class Func_t,class Arg_t>
typename std::enable_if<is_unary_functor<Func_t,Arg_t>::value,const char*>::type
function_identifier(Func_t f,Arg_t arg)
{
    return "its functor...";
}

template<class Func_t,class Arg_t>
typename std::enable_if<is_unary_function_ptr<Func_t,Arg_t>::value,const char*>::type
function_identifier(Func_t f,Arg_t arg)
{
    return "its function pointer...";
}

template<class Func_t,class Arg_t>
typename std::enable_if<!is_unary_function<Func_t,Arg_t>::value,const char*>::type
function_identifier(Func_t f,Arg_t arg)
{
    return "its not unary function...";
}
*/

template<class Func_t,class Iter_t>
concept unary_func=requires(Func_t fct,Iter_t iter){  fct(*iter); };

template<class Func_t,class Iter_t>
concept binary_func=requires(Func_t fct,Iter_t iter){  fct(*iter,*iter); };


template<class Func_t,class Iter_t>
requires unary_func<Func_t,Iter_t>
Iter_t min_element(Iter_t fst,Iter_t _end,Func_t cfunc)
{
    if (fst == _end) return fst;
    Iter_t res = fst;
    auto min_value=cfunc(*res);
    decltype(min_value) temp;
    while (++fst != _end)
    {
        temp=cfunc(*fst);
        if(temp<min_value)
        {
            min_value=temp;
            res=fst;
        }
    }
    return res;
}

template<class Func_t,class Iter_t>
requires binary_func<Func_t,Iter_t>
Iter_t min_element(Iter_t fst,Iter_t _end,Func_t cfunc)
{
    return std::min_element(fst,_end,cfunc);
}

template<class Func_t,class Iter_t>
requires unary_func<Func_t,Iter_t>
Iter_t max_element(Iter_t fst,Iter_t _end,Func_t cfunc)
{
    if (fst == _end) return fst;
    Iter_t res = fst;
    auto max_value=cfunc(*res);
    decltype(max_value) temp;
    while (++fst != _end)
    {
        temp=cfunc(*fst);
        if(max_value<temp)
        {
            max_value=temp;
            res=fst;
        }
    }
    return res;
}

template<class Func_t,class Iter_t>
requires binary_func<Func_t,Iter_t>
Iter_t max_element(Iter_t fst,Iter_t _end,Func_t cfunc)
{
    return std::max_element(fst,_end,cfunc);
}

// поиск минимакса

template<class Func_t,class Iter_t>
requires unary_func<Func_t,Iter_t>
std::pair<Iter_t,Iter_t> minmax_element(Iter_t fst,Iter_t _end,Func_t cfunc)
{
    Iter_t next=fst;
    if (fst == _end||++next==_end) return std::make_pair(fst,fst);
    Iter_t min_iter=fst;
    auto _min_value=cfunc(*min_iter);

    Iter_t max_iter=next;
    auto _max_value=cfunc(*max_iter);

    if(_max_value<_min_value)
    {
        std::swap(_min_value,_max_value);
        std::swap(min_iter,max_iter);
    }

    decltype(_min_value) temp;
    while (++fst != _end)
    {
        temp=cfunc(*fst);
        if(temp<_min_value)
        {
            _min_value=temp;
            min_iter=fst;
        }
        else if(_max_value<temp)
        {
            _max_value=temp;
            max_iter=fst;
        }
        else{}
    }
    return {min_iter,max_iter};
}

template<class Func_t,class Iter_t>
requires binary_func<Func_t,Iter_t>
std::pair<Iter_t,Iter_t> minmax_element(Iter_t fst,Iter_t _end,Func_t cfunc)
{
    return std::minmax_element(fst,_end,cfunc);
}

}// ext




#endif













