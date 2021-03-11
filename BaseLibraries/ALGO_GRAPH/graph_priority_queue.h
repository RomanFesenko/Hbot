
#ifndef  _graph_priority_queue_
#define  _graph_priority_queue_
#include "algoheap.h"
#include "property.h"
#include <vector>


//Priority queue,with
//support change priority operation

template <class node_t,class priority_t, class comp_fun_t>
class graph_priority_queue
{
   public:
   using ptr_node_t=const node_t*;
   private:
   std::vector<const node_t*> m_nodes;
   using heap_iterator_t=typename std::vector<const node_t*>::iterator;
   TProperty<priority_t,node_t>& m_priority;
   TProperty<heap_iterator_t,node_t> m_heap_iterators;
   public:
   comp_fun_t m_comp_fun;
   graph_priority_queue(TProperty<priority_t,node_t>& priority,comp_fun_t comp_fun):
   m_priority(priority),
   m_heap_iterators(priority.size()),
   m_comp_fun(comp_fun)
   {
       m_nodes.reserve(priority.size());
   }
   graph_priority_queue(const graph_priority_queue& other)=delete;
   graph_priority_queue& operator=(const graph_priority_queue& other)=delete;

   private:
   constexpr auto compare_ptrs()
   {
        return [this](const node_t* a,const node_t* b)
        {
            return m_comp_fun( m_priority(*a),m_priority(*b));
        };
   }

   constexpr auto swap_in_heap()
   {
       return [this](heap_iterator_t node_1,heap_iterator_t node_2 )
       {
           m_heap_iterators(**node_1)=node_2;
           m_heap_iterators(**node_2)=node_1;
           std::iter_swap(node_1,node_2);
       };
   }

   public:
   inline int size() const{return m_nodes.size(); }
   inline void clear(){m_nodes.clear();m_heap_iterators.clear();}
   inline bool empty()const{return m_nodes.empty();}

   void push(const node_t& node,priority_t setpriority)
   {
       m_nodes.push_back(&node);
       m_heap_iterators(node)=m_nodes.end()-1;
       m_priority(node)=setpriority;
       heap_insert(m_nodes.begin(),m_nodes.end(),compare_ptrs(),
                                                 swap_in_heap());
   }
   const node_t* top()const
   {
       return m_nodes.front();
   }
   void pop()
   {
       heap_erase_max(m_nodes.begin(),m_nodes.end(),compare_ptrs(),
                                                    swap_in_heap());
       m_nodes.pop_back();
   }
   // Caller side provide that new priority>old priority
   void inc_priority(const node_t& node,priority_t new_priority)
   {
       m_priority(node)=new_priority;
       heap_iterator_t old_iterator=m_heap_iterators(node);
       heapify_down_to_up(m_nodes.begin(),m_nodes.end(),old_iterator,
                          compare_ptrs(),
                          swap_in_heap());
   }
   // Caller side provide that new priority<old priority
   void dec_priority(const node_t& node,priority_t new_priority)
   {
       m_priority(node)=new_priority;
       heap_iterator_t old_iterator=m_heap_iterators(node);
       heapify_up_down(m_nodes.begin(),m_nodes.end(),
       old_iterator,compare_ptrs(),swap_in_heap());
   }
   void change_priority(const node_t& node,priority_t new_priority)
   {
       priority_t last=m_priority(node);
       if(m_comp_fun(last,new_priority))
       {
           inc_priority(node,new_priority);
       }
       else
       {
           dec_priority(node,new_priority);
       }
   }
};

#endif

