
#ifndef  _floyd_warshall_
#define  _floyd_warshall_
#include "../ALGO_EXTEND/matrix.h"
#include <type_traits>
#include <utility>
#include "edge_visitors.h"


template<class node_t>
class floyd_warshall_process
{
    const std::vector< node_t*>& m_graph_nodes;
    using weight_t= typename node_t::weight_t;
    using iterator= typename node_t::iterator;
    ext::matrix<weight_t> m_weights;
    ext::matrix<bool> m_is_path;
    ext::matrix<iterator> m_next_iterator;
    public:
    floyd_warshall_process(const std::vector< node_t*>& graph_nodes):
    m_graph_nodes(graph_nodes),
    m_weights(graph_nodes.size(),graph_nodes.size()),
    m_is_path(graph_nodes.size(),graph_nodes.size()),
    m_next_iterator(graph_nodes.size(),graph_nodes.size())
    {
        restore_original();
    }

    weight_t weight_zero(){return weight_t(0);}
    int num_nodes()const{return m_graph_nodes.size();}
    void done();

    void restore_original();

    weight_t weight_result(const node_t&from,const node_t&to_n)const
    {
        return m_weights[from.index()][to_n.index()];

    }
    bool is_path(const node_t&from,const node_t&to_n)const
    {
        return m_is_path[from.index()][to_n.index()];
    }
    template<class _Output>
    bool get_path(const node_t&from,const node_t&to_n,_Output output)const;
};


template <class node_t>
void floyd_warshall_process<node_t>::restore_original()
{
    // инициализация
    for(int i=0;i<num_nodes();i++)
    {
        m_weights[i][i]=weight_zero();
        m_is_path[i][i]=true;
        for(int j=i+1;j<num_nodes();j++)
        {
            //path i <-> j doesn't exist yet
            m_is_path[i][j]=false; m_is_path[j][i]=false;
        }
    }

    auto _edge_process=[this](const typename node_t::iterator& iter)
    {
        int i_from=iter.from_node().index();
        int i_to=iter.to_node().index();

        m_weights[i_from][i_to]=iter.weight();

        m_next_iterator[i_from][i_to]=iter;

        m_is_path[i_from][i_to]=true;
    };

    struct fwr_edge_visitor:public unique_edge_visitor<has_direct_edge_process|
                                                    has_undirect_edge_process|
                                                    has_bidirect_edge_process>
    {
        decltype(_edge_process) m_edge_process;

        fwr_edge_visitor(decltype(_edge_process) _process):
        m_edge_process(_process)
        {}

        void undirect_edge_process(const typename node_t::iterator& iter,
                                   const typename node_t::iterator& rev_iter)
        {
            m_edge_process(iter);
            m_edge_process(rev_iter);
        }

        void bidirect_edge_process(const typename node_t::iterator& iter,
                                   const typename node_t::iterator& rev_iter)
        {
            undirect_edge_process(iter,rev_iter);
        }

        void direct_edge_process(const typename node_t::iterator& iter)
        {
            m_edge_process(iter);
        }

    };

    fwr_edge_visitor temp(_edge_process);
    unique_edge_visit(m_graph_nodes,temp);
}


template <class node_t>
void floyd_warshall_process<node_t>::done()
{
    weight_t new_weight;
    // from j to k through i
    for(int i=0;i<num_nodes();i++)
    {
        for(int j=0;j<num_nodes();j++)
        {
            if(!m_is_path[j][i]) continue;
            for(int k=0;k<num_nodes();k++)
            {
                if(!m_is_path[i][k]) continue;
                new_weight=m_weights[j][i]+m_weights[i][k];
                if(!m_is_path[j][k])
                {
                    m_weights[j][k]=new_weight;
                    m_is_path[j][k]=true;
                    m_next_iterator[j][k]=m_next_iterator[j][i];
                }
                else
                {
                    if(new_weight<m_weights[j][k])
                    {
                       m_weights[j][k]=new_weight;
                       m_is_path[j][k]=m_is_path[j][i];
                       m_next_iterator[j][k]=m_next_iterator[j][i];
                    }
                }
            }
        }
    }
}


template <class node_t>
template <class _Output>
bool floyd_warshall_process<node_t>::get_path(const node_t&from,const node_t&to_n,
                                              _Output out_iter)const
{
    int i_from=from.index(); int i_to=to_n.index();
    if(!m_is_path[i_from][i_to]) return false;
    for(iterator it;i_from!=i_to;)
    {
        it=m_next_iterator[i_from][i_to];
        *out_iter=it;++out_iter;
        i_from=it.to_node().index();
    }
    return true;
}

#endif

