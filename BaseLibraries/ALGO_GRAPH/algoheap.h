
#ifndef  _algoheap_
#define  _algoheap_
#include <algorithm>
#include <queue>


// Algorithms for working with a heap with a special function
// permutations of elements in the heap
// Allows to implement the operation of changing the priority
// without first removing the element

// All elements down it - make heap
//


// All items below it - form a heap
// but it is optional
// heapify_up_down - Heapify

template<class _Iter,class comp_fun_t,class _Iter_swap>
void heapify_up_down(_Iter begin,_Iter end,_Iter it,comp_fun_t comp_fun,_Iter_swap swap_in_heap)
{
    auto iter_comp=[comp_fun](_Iter it1,_Iter it2)
    {
        return  comp_fun(*it1,*it2);
    };
    if(it<begin) return;
    _Iter left_child,right_child,temp_max;
    while(it<end)
    {
      temp_max=it;
      left_child=begin+2*(it-begin)+1;
      right_child=left_child+1;
      // If the right child does not go beyond
      // array boundaries are left and even more
      if(right_child<end)
      {
         temp_max=std::max({left_child,right_child,it},iter_comp);
      }
      else if(left_child<end)
      {
         temp_max=std::max(it,left_child,iter_comp);
      }
      else{ return;}

      if(it==temp_max)
      {
          return;
      }
      else
      {
          swap_in_heap(it,temp_max);
          it= temp_max;
      }
    }
}


// The it element is larger than its parent, violating the heap
// and goes up

template<class _Iter,class comp_fun_t,class _Iter_swap>
void heapify_down_to_up(_Iter begin,_Iter end,_Iter it ,comp_fun_t comp_fun,_Iter_swap swap_in_heap )
{
    _Iter parent;
    while(it>begin)
    {
        parent=begin+(it-begin-1)/2;
        if(comp_fun(*parent,*it))
        {
            swap_in_heap(parent,it);
            it=parent;continue;
        }
        else
        {
            return;
        }
    }
}


// end - newly inserted element

template<class _Iter,class comp_fun_t,class _Iter_swap>
void heap_insert(_Iter begin,_Iter end ,comp_fun_t comp_fun,_Iter_swap swap_in_heap)
{
    heapify_down_to_up(begin,end,end-1,comp_fun,swap_in_heap);
}

template<class _Iter,class comp_fun_t,class _Iter_swap>
void heap_erase_max(_Iter begin,_Iter end,comp_fun_t comp_fun,_Iter_swap swap_in_heap)
{
    swap_in_heap(begin,end-1);
    heapify_up_down( begin, end-1, begin,comp_fun,swap_in_heap);
}

#endif

