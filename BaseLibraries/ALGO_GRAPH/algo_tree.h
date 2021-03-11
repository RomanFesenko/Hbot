
#ifndef  _algo_tree_
#define  _algo_tree_

#include <memory>
#include <initializer_list>
#include <vector>

//определение наличия специфических обработчиков
// при обходе дерева

// обработчик узла при первом посещении
template<class node_t,class _Visitor>
concept def_first_visit=requires(_Visitor& visitor,node_t& node)
{
    visitor.first_visit(node);
};

// обработчик  ребра от предка к потомку
template<class node_t,class _Visitor>
concept def_edge_visit=requires(_Visitor&visitor,typename node_t::iterator& iter)
{
    visitor.edge_visit(iter);
};

// обработчик узла при последнем посещении
template<class node_t,class _Visitor>
concept def_last_visit=requires(_Visitor& visitor,node_t& node)
{
    visitor.last_visit(node);
};


const int has_first_visit=1<<0;
const int has_edge_visit=1<<1;
const int has_last_visit=1<<2;

template<int _Flag>
struct tree_visitor
{
    const static int _tree_process =_Flag;
};



// обход в порядке - все потомки - корень

template<class node_t,class _Visitor>
void postorder_tree_walk(node_t& root,_Visitor& visitor)
{
    static_assert(((_Visitor::_tree_process&has_first_visit)!=0)==
                 def_first_visit<node_t,_Visitor>);

    static_assert(((_Visitor::_tree_process&has_edge_visit)!=0)==
                 def_edge_visit<node_t,_Visitor>);

    static_assert(((_Visitor::_tree_process&has_last_visit)!=0)==
                 def_last_visit<node_t,_Visitor>);

    static_assert(_Visitor::_tree_process!=0,"Void tree process");


    using iterator=typename node_t::iterator;
    iterator temp_iter;

    if constexpr( def_first_visit<node_t,_Visitor>)
    {
        visitor.first_visitor(root);
    }

    temp_iter.set_base(root);
    std::vector<iterator> iter_stack;
    iter_stack.push_back(temp_iter);
    // максимальный рамер стека==высоте дерева
    // независимо от ветвления на каждом уровне
    while(!iter_stack.empty())
    {
        // все потомки iter_stack.back().from_node()
        // обработаны
        if(iter_stack.back().end())
        {
            if constexpr(def_last_visit<node_t,_Visitor>)
            {
                visitor.last_visit(iter_stack.back().from_node());
            }
            iter_stack.pop_back();
            continue;
        }

        //обработка следующего потомка если ребро действительно
        // (при наличии обработчика ребра)
        if constexpr(def_edge_visit<node_t,_Visitor>)
        {
            if(!visitor.edge_visit(iter_stack.back()))
            {
                iter_stack.back().next();
                continue;
            }
        }

        node_t& next=iter_stack.back().to_node();
        if constexpr( def_first_visit<node_t,_Visitor>)
        {
            visitor.first_visit(next);
        }
        temp_iter.set_base(next);
        iter_stack.push_back(temp_iter);
        (iter_stack.end()-2)->next();
    }
}


template<class derived_node_t>
class CTreeNode
{
    public:
    using node_ptr_t=std::unique_ptr<derived_node_t>;
    protected:
    std::vector<node_ptr_t> m_childs;
    public:
    CTreeNode(){}
    CTreeNode(std::initializer_list<derived_node_t*> childs)
    {
        for(derived_node_t* child:childs)
        {
            m_childs.push_back(node_ptr_t(child));
        }
    }
    class iterator
    {
        typename std::vector<node_ptr_t>::iterator m_current;
        derived_node_t* m_base;
        public:
        void set_base(derived_node_t&node)
        {
            m_base=&node;
            m_current=node.m_childs.begin();
        }

        derived_node_t& from_node()const{return *m_base;}

        derived_node_t& to_node()const{return *(m_current->get());}

        void begin(){m_current=m_base->m_childs.begin();}
        void next(){++m_current;}
        bool end()const{return m_current==m_base->m_childs.end();}
    };
    friend class iterator;
};

#endif

