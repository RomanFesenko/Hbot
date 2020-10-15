
#ifndef  _bellman_ford_
#define  _bellman_ford_
#include <vector>
#include <deque>

#include "property.h"
#include "search_adaptors.h"


template<class node_t>
class bellman_ford_process
{
    const std::vector< node_t*>& m_graph_nodes;
    TProperty<char,node_t> m_nodes_state;
    enum State{Sleep=0,Alive,Killed};
    public:
    using ptr_node_t=node_t*;
    using weight_t= typename node_t::weight_t;
    using iterator= typename node_t::iterator;//итератор обхода смежных
    // Каждому узлу сопоставляем вес,
    TProperty<weight_t,node_t> m_weights;
    // итератор - ребро ведущее из предыдущей вершины
    TProperty<iterator,node_t> m_prev_iterator;//prev_node[node.index()] - предыдущий для node
    bellman_ford_process(const std::vector< node_t*>& graph_nodes)
    :m_graph_nodes(graph_nodes),
    m_nodes_state(m_graph_nodes.size()),
    m_weights(m_graph_nodes.size()),
    m_prev_iterator(m_graph_nodes.size()){}

    int num_nodes()const{return m_graph_nodes.size();}
    weight_t weight_zero(){return weight_t(0);}
    bool get_path(const node_t& from,const node_t& to,std::vector<iterator>& iters)const;

    template<class _Restr>
    bool done(const node_t&,_Restr restrictor);
    bool done(const node_t& from)
    {
        struct s_void{} default_void;
        return done(from,default_void);
    }
    weight_t weight_result(const node_t& node)const
    {
        return m_weights(node);

    }
    bool is_find(const node_t& node)const
    {
        return m_nodes_state(node)==Alive;
    }
};

// Поиск кратчайшИХ пути в ориентированом графе
// допускаются ребра с отрицательным весом
// возвращает false если есть отрицательные циклы
template<class node_t>
template<class _Restr>
bool bellman_ford_process<node_t>:: done(const node_t& start_node,_Restr restrictor)
{
   std::deque<const node_t*> for_process;
   std::fill(m_nodes_state.begin(),m_nodes_state.end(),Sleep);

   if constexpr(def_node_discover<_Restr,node_t>)
    {
        if(restrictor.node_discover(start_node))
        {
            for_process.push_back(&start_node);
            m_weights(start_node)=weight_zero();
            m_nodes_state(start_node)=Alive;
        }
        else
        {
            m_nodes_state(start_node)=Killed;
        }
    }
    else
    {
         for_process.push_back(&start_node);
         m_weights(start_node)=weight_zero();
         m_nodes_state(start_node)=Alive;
    }
   for_process.push_back(nullptr);
   int num_series=-1;
   while(true)
   {
       const node_t*next_for_process=for_process.front();
       for_process.pop_front();
       if(next_for_process==nullptr)//сигнал конца текущей обрабатываемой серии
       {
           if(++num_series>num_nodes()) return true;
           for_process.push_back(nullptr);
           continue;
       }
       // обработка смежных нодов- стандартный алгоритм релаксации
       // с той разницей что если удается уменьшить
       // какой-то путь на m_num_nodes итерации -  это отриц цикл
       iterator temp_it;
       temp_it.set_base(*next_for_process);
       for(temp_it.begin();!temp_it.end();temp_it.next())
       {
           // возможно  ребро спрятано
           if constexpr(def_unary_edge_process<_Restr,node_t>)
           {
               if(!restrictor.is_edge_process(temp_it)) continue;
           }
           weight_t new_weight=m_weights(*next_for_process)+temp_it.weight();
           const node_t& adj_for_process=temp_it.to_node();
           if(m_nodes_state(adj_for_process)==Sleep)
           {
               // возможно смежный нод спрятан
               if constexpr(def_node_discover<_Restr,node_t>)
               {
                   if(!restrictor.node_discover(adj_for_process))
                   {
                       m_nodes_state(adj_for_process)=Killed;
                       continue;
                   }
                   m_weights(adj_for_process)=new_weight;
                   m_nodes_state(adj_for_process)=Alive;
                   m_prev_iterator(adj_for_process)=temp_it;
                   for_process.push_back(&adj_for_process);
               }
               else
               {
                    m_weights(adj_for_process)=new_weight;
                    m_nodes_state(adj_for_process)=Alive;
                    m_prev_iterator(adj_for_process)=temp_it;
                    for_process.push_back(&adj_for_process);
               }

           }
           else if(m_nodes_state(adj_for_process)==Alive)// релаксация
           {
               //если после обработки m_num_nodes-1
               // серий попадается возможность ослабить ребро - цикл
               if(new_weight<m_weights(adj_for_process))
               {
                   if(num_series==num_nodes()) return false;
                   m_weights(adj_for_process)=new_weight;
                   m_prev_iterator(adj_for_process)=temp_it;
                   for_process.push_back(&adj_for_process);
               }

           }
           else{}// убитый
       }
   }
}

template<class node_t>
bool bellman_ford_process<node_t>::get_path(const node_t& start_node,const node_t& end_node,std::vector<iterator>& iters)const
{
    iters.clear();
    if(!is_find(end_node)) return false;
    const node_t* from_n=&start_node;
    const node_t* to_n=&end_node;
    iterator it;
    while(to_n!=from_n)
    {
        it=m_prev_iterator(*to_n);
        iters.push_back(it);
        to_n=&it.from_node();
    }
    std::reverse(iters.begin(),iters.end());
    return true;
}

#endif

