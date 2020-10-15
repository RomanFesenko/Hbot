
#ifndef  _algoheap_
#define  _algoheap_
#include <algorithm>
#include <queue>


// Алгоритмы работы с пирамидой со специальной функцией
// перестановки элементов в пирамиде
// Позволяет реализовать операцию изменения приоритета
// без предварительного удаления элемента

// Все єлементы ниже it - образуют пирамиду
// но it не обязательно
// heapify_up_down - перестраивает в виде пирамиды

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
      //Если правый потомок не выходит за
      // границы массива то левый и подавно
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

// Элемент it больше своего родителя нарушая св-во пирамиды
// и поднимается вверх

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

//end -вновь вставленый элемент

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

