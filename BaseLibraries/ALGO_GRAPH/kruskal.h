
#ifndef  _kruskal_
#define  _kruskal_
#include <algorithm>
#include <numeric>

#include <utility>
#include <vector>
#include "edge_visitors.h"


template <class node_t,class _Output_iterator>
typename node_t::weight_t kruskal(const std::vector<node_t*>&nodes,
_Output_iterator output)
{
    typedef typename node_t::iterator iterator;
    typedef typename node_t::weight_t weight_t;
    struct edge_extractor_t:public unique_edge_visitor<has_undirect_edge_process>
    {
        std::vector<iterator>& m_queue;
        edge_extractor_t(std::vector<iterator>&queue):
        m_queue(queue){}

        void undirect_edge_process(const iterator& iter)
        {
            m_queue.push_back(iter);
        }
    };
    auto edge_comp=[](const iterator& a,const iterator& b)
    {
          return b.weight()<a.weight();
    };
    std::vector<iterator> edges;
    edge_extractor_t edge_extractor(edges);
    unique_edge_visit(nodes,edge_extractor);

    std::make_heap(edges.begin(),edges.end(),edge_comp);


    std::vector<int> unions; unions.resize(nodes.size());
    std::iota(unions.begin(),unions.end(),0);

    std::vector<int> weights(nodes.size(),1);

    auto find_root=[&unions](int el)
    {
        int i;
        for(i=el;i!=unions[i];i=unions[i]);
        return i;
    };
    // быстрое взвешеное объединение
    auto quw_union=[&weights,find_root,&unions](int fst_el,int snd_el)
    {
        int fst_root=find_root(fst_el);
        int snd_root=find_root(snd_el);

        if(fst_root==snd_root) return false;// оба эл-та Є одному дереву

        // меньшее привяз. к большему
        if(weights[snd_root]<weights[fst_root])
        {
            unions[snd_root]=fst_root;
            weights[fst_root]+=weights[snd_root];
        }
        else
        {
            unions[fst_root]=snd_root;
            weights[snd_root]+=weights[fst_root];
        }
        return true;
    };
    iterator temp;
    weight_t total_weight=weight_t(0);

    //Последовательно вытаскиваем из пирамиды самые
    // легкие ребра
    typename std::vector<node_t*>::size_type num_tree_edges=0;
    while((!edges.empty())&&(num_tree_edges<nodes.size()))
    {
        std::pop_heap(edges.begin(),edges.end(),edge_comp);
        temp=edges.back();edges.pop_back();
        //cout<<"weight: "<<temp.weight()<<"\n";
        // используем алг быстрого вэвешеного
        // обЪеденения чтобы выяснить принадлежат ли
        // вершины разным деревьям
        if(quw_union(temp.from_node().index(),temp.to_node().index()))
        {
            *output=temp; ++output;
            total_weight+=temp.weight();
            ++num_tree_edges;
        }
    }
    return total_weight;
}


#endif

