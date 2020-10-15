
#ifndef  _priority_graph_search_
#define  _priority_graph_search_

#include <assert.h>

#include <vector>
#include "property.h"
#include "graph_priority_queue.h"
#include "search_adaptors.h"

// интерфейс итератора нода
// it.set_base(const node& nd);
// it.begin()
// it.end()
// it.next()
// const node& it.from_node()
// const node& it.to_node()
// weight_t it.weight()


// в алгоритмах Дейкстры ,Прима используется
// процедура релаксации с разным списком аргументов

template<class node_t,class _Relax_func_t>
concept unary_relaxer=requires(const node_t::iterator& iter,_Relax_func_t relax)
{
    relax(iter);
};

template<class node_t,class _Relax_func_t>
concept binary_relaxer=requires(const node_t::iterator& iter,
                                node_t::weight_t weight,
                                _Relax_func_t relax)
{
    relax(iter,weight);
};


// обобщенный поиск на графе с использованием очереди с приоритетом
// с  поддержкой произвольной прцедуры релаксации ребер
// и фильтров ребер/узлов

// частные случаи - алг. Дейкстры,Прима,варианты поиска аугментального пути
// в алг. Форда - Фалкерсона

template<class node_t>
class priority_search
{
    protected:
    const std::vector< node_t*>& m_graph_nodes;
    typename std::vector<node_t*>::size_type m_num_nodes;
    TProperty<char,node_t> m_nodes_state;
    enum State{Sleep=0,Alive,Dead,Killed};
    public:
    using ptr_node_t=node_t*;
    using weight_t= typename node_t::weight_t;
    using iterator= typename node_t::iterator;//итератор обхода смежных
    // Каждому узлу сопоставляем вес,
    TProperty<weight_t,node_t> m_weights;
    // итератор - ребро ведущее из предыдущей вершины
    TProperty<iterator,node_t> m_prev_iterator;//prev_node[node.index()] - предыдущий для node

    priority_search(const std::vector< node_t*>& graph_nodes)
    :m_graph_nodes(graph_nodes),m_num_nodes(graph_nodes.size()),
    m_nodes_state(m_num_nodes),m_weights(m_num_nodes),
    m_prev_iterator(m_num_nodes){}

    weight_t weight_zero(){return weight_t(0);}

    template<class _Restr,class _Relax_func_t>
    void done(const node_t& from,_Restr& restrictor,_Relax_func_t);
    weight_t weight_result(const node_t& node)const
    {
        return m_weights(node);

    }
    bool is_find(const node_t& node)const
    {
        return m_nodes_state(node)==Dead;
    }
};

/* relaxer - функция ослабления ребра

 Иинтерфейс restrictor (наличие любой функции-члена опционально):
 restrictor.node_discover->bool - визитор нода при первом посещении,
 если вовращает false, нод исключается из поиска;
 restrictor.is_edge_process->bool - визитор ребра,если вовращает
 false, ребро исключается из поиска;
 restrictor.is_node_finish->bool визитор нода при последнем посещении,
 если вовращает true,поиск прекращается;

 restrictor должен наследовать от search_restrictor типа

 CRestrictor:public search_restrictor<has_node_discover|
                                      has_node_finish|
                                      ....>

*/

template<class node_t>
template<class _Restr,class _Relax_func_t>
void priority_search<node_t>::done(const node_t& start_node,_Restr& restrictor,
                                        _Relax_func_t relaxer)
{
    // страховка от неправильного объявления визиторов в restrictor-
    // ассерт сработает если отсутсвует визитор при наличии
    // соответствующего флага в _specific_restriction отнаследованого
    // от search_restrictor

    static_assert(((_Restr::_specific_restriction&has_node_discover)!=0)==
                 def_node_discover<_Restr,node_t>);

    static_assert(((_Restr::_specific_restriction&has_unary_edge_process)!=0)==
                 def_unary_edge_process<_Restr,node_t>);

    static_assert(((_Restr::_specific_restriction&has_binary_edge_process)!=0)==
                 def_binary_edge_process<_Restr,node_t>);


    static_assert(((_Restr::_specific_restriction&has_node_finish)!=0)==
                 def_is_node_finish<_Restr,node_t>);

    //static_assert(_Restr::_specific_restriction!=0);

    // Должен быть определен лишь один из
    static_assert((!def_binary_edge_process<_Restr,node_t>)||
                  (!def_unary_edge_process<_Restr,node_t>));

    // между вызовами  не должно меняться
    // число узлов графа
    assert(m_num_nodes==m_graph_nodes.size());
    std::fill(m_nodes_state.begin(),m_nodes_state.end(),Sleep);

    // следующим из очереди извлекается узел с наименьшим весом
    auto fpriority=[](weight_t a,weight_t b){return a > b;};

    graph_priority_queue<node_t,weight_t, decltype(fpriority)>
    queue(m_weights,fpriority);///

    if constexpr(def_node_discover<_Restr,node_t>)
    {
        if(restrictor.node_discover(start_node))
        {
            queue.push(start_node,weight_zero());
            m_nodes_state(start_node)=Alive;
        }
        else
        {
            m_nodes_state(start_node)=Killed;
        }
    }
    else
    {
         queue.push(start_node,weight_zero());
         m_nodes_state(start_node)=Alive;
    }
    iterator it;
    while(!queue.empty())
    {
        const node_t& new_dead=*queue.top();queue.pop();

        /*cout<< "m_weights(new_dead): "<<m_weights(new_dead)<<endl;
        system("PAUSE");*/

        m_nodes_state(new_dead)=Dead;
        if constexpr(def_is_node_finish<_Restr,node_t>)
        {
             if(restrictor.is_node_finish(new_dead)) break;
        }
        it.set_base(new_dead);
        // обработка всех исходящих ребер
        for(it.begin();!it.end();it.next())
        {
            // процедура релаксации исходящих ребер
            weight_t new_weight;
            if constexpr(unary_relaxer<node_t,_Relax_func_t>)
            {
                new_weight=relaxer(it);//прим
            }
            else// дейкстра или поиск аугментального пути
            {
                new_weight=relaxer(it,m_weights(it.from_node()));
            }

            // подлежит  ли обработке это ребро
            if constexpr(def_unary_edge_process<_Restr,node_t>)
            {
                if(!restrictor.is_edge_process(it)) continue;
            }
            else if constexpr(def_binary_edge_process<_Restr,node_t>)
            {
                if(!restrictor.is_edge_process(it,new_weight))continue;
            }
            else{}
            // подлежит
            const node_t& to_n=it.to_node();
            if(m_nodes_state(to_n)==Alive)
            {
                if(new_weight<m_weights(to_n))
                {
                    queue.inc_priority(to_n,new_weight);
                    m_prev_iterator(to_n)=it;
                }
            }
            else if(m_nodes_state(to_n)==Sleep) // спящий нод, будим
            {
                if constexpr(def_node_discover<_Restr,node_t>)
                {
                    // фильтруем узел
                    if(restrictor.node_discover(to_n))
                    {
                        queue.push(to_n,new_weight);
                        m_prev_iterator(to_n)=it;
                        m_nodes_state(to_n)=Alive;
                    }
                    else
                    {
                        m_nodes_state(to_n)=Killed;
                    }
                }
                else
                {
                    queue.push(to_n,new_weight);
                    m_prev_iterator(to_n)=it;
                    m_nodes_state(to_n)=Alive;
                }
            }
            else {}// мертвый или убитый
        }// for
    } //while
}

//PRIM
    /*#1
    link_vec.push_back(Link(0,1,0.08,DOUBLE_LINK));
    link_vec.push_back(Link(0,2,0.99,DOUBLE_LINK));
    link_vec.push_back(Link(0,5,0.65,DOUBLE_LINK));
    link_vec.push_back(Link(0,6,0.39,DOUBLE_LINK));
    link_vec.push_back(Link(0,7,0.49,DOUBLE_LINK));

    link_vec.push_back(Link(1,7,0.28,DOUBLE_LINK));

    link_vec.push_back(Link(3,4,0.65,DOUBLE_LINK));
    link_vec.push_back(Link(3,5,0.37,DOUBLE_LINK));

    link_vec.push_back(Link(4,5,0.78,DOUBLE_LINK));
    link_vec.push_back(Link(4,6,0.01,DOUBLE_LINK));
    link_vec.push_back(Link(4,7,0.12,DOUBLE_LINK));

    link_vec.push_back(Link(6,7,0.65,DOUBLE_LINK));*/
    // #2
    /*link_vec.push_back(Link(0,1,1,DOUBLE_LINK));
    link_vec.push_back(Link(0,2,5,DOUBLE_LINK));
    link_vec.push_back(Link(0,3,7,DOUBLE_LINK));
    link_vec.push_back(Link(0,4,9,DOUBLE_LINK));

    link_vec.push_back(Link(1,2,6,DOUBLE_LINK));
    link_vec.push_back(Link(1,3,4,DOUBLE_LINK));
    link_vec.push_back(Link(1,4,3,DOUBLE_LINK));

    link_vec.push_back(Link(2,3,5,DOUBLE_LINK));
    link_vec.push_back(Link(2,5,10,DOUBLE_LINK));

    link_vec.push_back(Link(3,4,8,DOUBLE_LINK));
    link_vec.push_back(Link(3,5,3,DOUBLE_LINK));*/

    /*std::pair<int,int>   links[6];
    value_type wres=Graph.prim(2,links);
    printf(" %s %f %s","weight:",wres,"\n");
    for_each(links,links+numn,[](auto& el)
             {
                 printf("%d %s %d %s",el.first," ",el.second,"\n");
             });*/
#endif

