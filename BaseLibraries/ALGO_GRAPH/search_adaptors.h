
#ifndef  _search_adaptors_
#define  _search_adaptors_


// the behavior of the algorithm depends on the presence
// special member functions in the search restrictor


// presence of a handler for first visited nodes
template<class _Restr,class node_t>
concept def_node_discover=requires(_Restr& res,const node_t& node)
{
    res.node_discover(node);
};

// presence of a handler for unary edge handler
template<class _Restr,class node_t>
concept def_unary_edge_process=requires(_Restr& res,const typename node_t::iterator& iter)
{
    res.is_edge_process( iter);
};

// presence of a handler for binary edge handler
template<class _Restr,class node_t>
concept def_binary_edge_process=requires
(_Restr& res,const typename node_t::iterator& iter,typename node_t::weight_t relax_w)
{
    res.is_edge_process( iter, relax_w);
};

// presence of  the finalizer
template<class _Restr,class node_t>
concept def_is_node_finish=requires(_Restr& res,const node_t& nde)
{
    res.is_node_finish(nde);
};

const int has_node_discover=1<<0;
const int has_unary_edge_process=1<<1;
const int has_binary_edge_process=1<<2;
const int has_node_finish=1<<3;
const int has_dead_node_process=1<<4;



template<int _Flag>
struct search_restrictor
{
    const static int _specific_restriction =_Flag;
};


// Most often, search_restrictor can be provided
// a combination of independent functors so that
// can be multiple inherited from _base_node_filter,_base_edge_filter

template<class..._Mixin>
struct _base_restrictor:public _Mixin...
{
    const static int _specific_restriction=
    (_Mixin::_specific_restriction|...|0);

    _base_restrictor(_Mixin...mix):_Mixin(mix)...{}

    auto get_sub_classes()
    {
        return make_tuple(_Mixin(_Mixin::get_functor())...);
    }

    template<class _Added>
    auto operator|(_Added added);
};

template<class...Mixins>
_base_restrictor<Mixins...> get_restrictor(Mixins...mix)
{
    return _base_restrictor<Mixins...>(mix...);
}

template<class...Mixins>
_base_restrictor<Mixins...> get_restrictor_from_tuple(std::tuple<Mixins...> _tuple)
{
    return std::apply(get_restrictor<Mixins...>,_tuple);
}

template<class...Mixins>
template<class _Added>
auto _base_restrictor<Mixins...>::operator|(_Added add)
{
    return get_restrictor_from_tuple
    (std::tuple_cat(get_sub_classes(),std::make_tuple(add)));
}


template<class node_t,class _Finder>
class _base_finder
{
    using self_t=_base_finder<node_t,_Finder>;

    _Finder m_finder;
    public:
    const static int _specific_restriction=has_node_finish;
    const node_t* goal_node=nullptr;

    _base_finder(_Finder finder):m_finder(finder){}

    bool is_node_finish(const node_t& node)
    {
        if(m_finder(node))
        {
            goal_node=&node;
            return true;
        }
        return false;
    }

    template<class _Filter>
    _base_restrictor<self_t,_Filter>
    operator|(_Filter filter)
    {
        return _base_restrictor<self_t,_Filter>(*this,filter);
    }

    _Finder get_functor(){return m_finder;}
};


template<class node_t,class _Filter>
class _base_node_filter
{
    _Filter m_node_filter;
    public:
    const static int _specific_restriction=has_node_discover;
    explicit _base_node_filter(_Filter _filter):m_node_filter(_filter){}
    bool node_discover(const node_t& node)
    {
        return m_node_filter(node);
    }

    _Filter get_functor(){return m_node_filter;}
};

template<class node_t,class _Filter>
struct _base_edge_filter
{
    using iterator=typename node_t::iterator;
    using weight_t=typename node_t::weight_t;

    _Filter m_edge_filter;
    public:
    const static int _specific_restriction=has_binary_edge_process;
    explicit _base_edge_filter(_Filter _filter):m_edge_filter(_filter){}
    bool is_edge_process(const iterator&iter,weight_t weight)
    {
        return m_edge_filter(iter,weight);
    }

    _Filter get_functor(){return m_edge_filter;}
};


template<class node_t,class _Finder>
_base_finder<node_t,_Finder>
search_terminator(_Finder finder)
{
    return _base_finder<node_t,_Finder>(finder);
}

template<class node_t,class _Filter>
_base_node_filter<node_t,_Filter>
node_filter(_Filter filt)
{
    return _base_node_filter<node_t,_Filter>(filt);
}

template<class node_t,class _Filter>
_base_edge_filter<node_t,_Filter>
edge_filter(_Filter filt)
{
    return _base_edge_filter<node_t,_Filter>(filt);
}

#endif

