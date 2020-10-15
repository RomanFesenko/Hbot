
#ifndef  _transpose_graph_
#define  _transpose_graph_
#include "edge_visitors.h"
#include "graph_composer.h"
#include "property.h"

// make_transpose - построение транспонированого графа

template<class source_node_t,
         class node_factory_t,
         class edge_factory_t,
         class result_node_t>
void make_transpose(const std::vector< source_node_t*>& source,
                    node_factory_t node_factory,
                    edge_factory_t edge_factory,
                    CGraphComposer<result_node_t>& transposer)
{
    using sce_iterator_t=source_node_t::iterator;

    TProperty<result_node_t*,source_node_t> result_node(source.size());


    struct sce_visitor_t:public unique_edge_visitor<has_undirect_edge_process|
                                                  has_direct_edge_process|
                                                  has_bidirect_edge_process>
    {
        TProperty<result_node_t*,source_node_t>& m_result_node;
        edge_factory_t& m_edge_factory;
        CGraphComposer<result_node_t>& m_transposer;

        sce_visitor_t(TProperty<result_node_t*,source_node_t>&result_node_,
                      edge_factory_t& edge_factory_,
                      CGraphComposer<result_node_t>& transposer_):
        m_result_node(result_node_),
        m_edge_factory(edge_factory_),
        m_transposer(transposer_)
        {}

         void undirect_edge_process(const sce_iterator_t& sce_iter)
         {
             m_transposer.set_undirect_edge(*m_result_node(sce_iter.to_node()),
                                            *m_result_node(sce_iter.from_node()),
                                            m_edge_factory(sce_iter.edge()));
         }
         void direct_edge_process(const sce_iterator_t& sce_iter)
         {
             m_transposer.set_direct_edge(*m_result_node(sce_iter.to_node()),
                                          *m_result_node(sce_iter.from_node()),
                                          m_edge_factory(sce_iter.edge()));
         }
         void bidirect_edge_process(const sce_iterator_t& iter_1_2,
                                    const sce_iterator_t& iter_2_1)
         {
             m_transposer.set_bidirect_edge(*m_result_node(iter_1_2.to_node()),
                                            *m_result_node(iter_1_2.from_node()),
                                            m_edge_factory(iter_1_2.edge()),
                                            m_edge_factory(iter_2_1.edge()));
         }
    };

    transposer.delete_graph();
    for(const source_node_t*sce_cur:source)
    {
        result_node_t* res_cur=node_factory(*sce_cur);
        result_node(*sce_cur)=res_cur;
        transposer.add_node(res_cur);
    }

    sce_visitor_t sce_visitor(result_node,edge_factory,transposer);
    unique_edge_visit(source,sce_visitor);
}

#endif

