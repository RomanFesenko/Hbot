
#ifndef  _edge_visitors_
#define  _edge_visitors_
#include <vector>
#include <assert.h>


// determine if there are specific handlers for
// of each edge type

template<class node_t,class _Visitor>
concept unary_undirect_edge_process=requires(_Visitor& reader,
const typename node_t::iterator& iter)
{
    reader.undirect_edge_process(iter);
};

template<class node_t,class _Visitor>
concept binary_undirect_edge_process=requires(_Visitor& reader,
const typename node_t::iterator& iter)
{
    reader.undirect_edge_process(iter,iter);
};

// undirected edge handler
template<class node_t,class _Visitor>
concept def_undirect_edge_process=
unary_undirect_edge_process<node_t,_Visitor>||
binary_undirect_edge_process<node_t,_Visitor>;


// directed edge handler
template<class node_t,class _Visitor>
concept def_direct_edge_process=requires(_Visitor& reader,
const typename node_t::iterator& iter)
{
    reader.direct_edge_process(iter);
};

//2-way edge handler
template<class node_t,class _Visitor>
concept def_bidirect_edge_process=requires(_Visitor& reader,
const typename node_t::iterator& iter12,
const typename node_t::iterator& iter21)
{
    reader.bidirect_edge_process(iter12,iter21);
};


const int has_undirect_edge_process=1<<0;
const int has_direct_edge_process=1<<1;
const int has_bidirect_edge_process=1<<2;

template<int _Flag>
struct unique_edge_visitor
{
    const static int _specific_edge_process =_Flag;
};

/**
 traversal of all edges of the graph with specific visitors
 for each type of edge; Visitors may attend
 or absent in any combination, but must be
 at least one.
 To prevent errors when declaring handlers,
 visitor inherits from unique_edge_visitor with a given set of flags
*/

template<class node_t,class _Visitor>
void unique_edge_visit(const std::vector< node_t*>& nodes,_Visitor& visitor)
{
    static_assert(((_Visitor::_specific_edge_process&has_undirect_edge_process)!=0)==
                 def_undirect_edge_process<node_t,_Visitor>);

    static_assert(((_Visitor::_specific_edge_process&has_direct_edge_process)!=0)==
                 def_direct_edge_process<node_t,_Visitor>);

    static_assert(((_Visitor::_specific_edge_process&has_bidirect_edge_process)!=0)==
                 def_bidirect_edge_process<node_t,_Visitor>);

    static_assert(_Visitor::_specific_edge_process!=0,"Void edge process");

    using iterator=typename node_t::iterator;
    iterator temp_it;
    for(const node_t*n_1:nodes)
    {
        temp_it.set_base(*n_1);
        // handling adjacent edges
        for(temp_it.begin();!temp_it.end();temp_it.next())
        {
            const node_t& n_2=temp_it.to_node();
            auto rev_iter=temp_it.get_reverse_iterator();

            // undirected or 2-way edge was
            // processed earlier
            if(n_2.index()<n_1->index()&&rev_iter) continue;

            if(!rev_iter)// directed edge
            {
                if constexpr(def_direct_edge_process<node_t,_Visitor>)
                {
                    visitor.direct_edge_process(temp_it);
                    continue;
                }
            }
            else if(&(rev_iter->edge())==&temp_it.edge())//undirected
            {
                if constexpr(unary_undirect_edge_process<node_t,_Visitor>)
                {
                    visitor.undirect_edge_process(temp_it);
                    continue;
                }
                else if constexpr(binary_undirect_edge_process<node_t,_Visitor>)
                {
                    visitor.undirect_edge_process(temp_it,rev_iter.value());
                    continue;
                }
                else{}
            }
            else//2-way
            {
                if constexpr(def_bidirect_edge_process<node_t,_Visitor>)
                {
                    visitor.bidirect_edge_process(temp_it,rev_iter.value());
                }
            }
        }
    }
}


#endif

