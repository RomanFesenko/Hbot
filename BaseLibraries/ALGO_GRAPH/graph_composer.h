
#ifndef  _graph_composer_
#define  _graph_composer_
#include <type_traits>
#include <assert.h>
#include <algorithm>
#include <vector>
#include <tuple>
#include <memory>
#include <optional>

enum TypeEdge
{
   id_no_edge=0,//00
   id_1_2_edge,//01
   id_2_1_edge,//10
   id_undirect_edge,// 11
   id_bidirect_edge
};


template<class node_t>
class CBaseNodeIterator;

template<class node_t>
class CGraphComposer;

// CBaseNode is the base class in the curiosly recursive pattern model:
// class CDerivedNode: public CBaseNode <some_edge_t, CDerivedNode>
// The derived class gets the CBaseNode interface to use
// CGraphComposer

template<class  ed_t,class derived_node_t>
class CBaseNode
{
   public:
   CBaseNode(const CBaseNode& node)=delete;
   CBaseNode& operator=(const CBaseNode& node)=delete;
   CBaseNode(){}
   using node_t=derived_node_t;
   using edge_t=ed_t;
   using iterator=CBaseNodeIterator<node_t>;
   private:
   int m_index;// index in graph
   // adjacent available
   struct adj_value_type
   {
       node_t* node;
       std::shared_ptr<edge_t> edge;
   };
   std::vector<adj_value_type> m_adjacents;
   using adj_iterator=typename decltype(m_adjacents)::iterator;
   using const_adj_iterator=typename decltype(m_adjacents)::const_iterator;

   //adjacent non-available
   std::vector<node_t*> m_adj_nonavailables;
   using node_iterator=typename decltype(m_adj_nonavailables)::iterator;
   struct _adj_comp_node
   {
        bool operator()(const adj_value_type& adj1,
                        const node_t& adj2)const
        {
            return adj1.node < &adj2;
        }
   };
   // delete_edge,set_edge -  available only for
   // CGraphComposer
   const edge_t* delete_edge(const node_t& other)
   {
       auto opt_iter=std::lower_bound(m_adjacents.begin(),
                                      m_adjacents.end(),
                                      other,_adj_comp_node());
       if(opt_iter!=m_adjacents.end()&&opt_iter->node==&other)
       {
           const edge_t* for_return=opt_iter->edge.get();
           m_adjacents.erase(opt_iter);
           return for_return;
       }
       else
       {
           auto opt_iter_2=std::lower_bound(m_adj_nonavailables.begin(),
                                            m_adj_nonavailables.end(),
                                            &other);
           if(opt_iter_2!=m_adj_nonavailables.end()&&*opt_iter_2==&other)
           {
               m_adj_nonavailables.erase(opt_iter_2);
           }
       }
       return nullptr;
   }
   void set_edge(node_t& other,const std::shared_ptr<edge_t>& egd)
   {
       auto adj_iter=std::lower_bound(m_adjacents.begin(),
                                      m_adjacents.end(),
                                      other,_adj_comp_node());
       if(adj_iter!=m_adjacents.end())
       {
           assert(adj_iter->node!=&other);
       }
       m_adjacents.insert(adj_iter,{&other,egd});
   }
   void add_nonavailable(node_t& other)
   {
       auto adj_iter=std::lower_bound(m_adj_nonavailables.begin(),
                                      m_adj_nonavailables.end(),&other);
       if(adj_iter!=m_adj_nonavailables.end())
       {
           assert(*adj_iter!=&other);
       }
       m_adj_nonavailables.insert(adj_iter,&other);
   }

   public:
   using adj_size_type=typename decltype(m_adjacents)::size_type;
   using weight_t=decltype(m_adjacents[0].edge->weight()+0);
   adj_size_type size_adjacents()const{return m_adjacents.size();}
   edge_t* get_edge_to(const node_t& other)const
   {
       auto adj_iter=std::lower_bound(m_adjacents.begin(),
                                      m_adjacents.end(),
                                      other,_adj_comp_node());
       return (adj_iter!=m_adjacents.end()&&adj_iter->node==&other)?
       adj_iter->edge.get() : nullptr;
   }

   std::optional<iterator> get_iterator_to(const node_t& other)const;

   bool is_adjacent(const node_t& other)const
   {
       return get_edge_to(other)!=nullptr;
   }
   const std::vector<node_t*>& not_availables()const
   {
       return m_adj_nonavailables;
   }

   int index()const{return m_index;}
    ~CBaseNode(){}
    friend class CGraphComposer<node_t>;
    friend class CBaseNodeIterator<node_t>;
};


// CBaseNodeIterator
// Provides the minimum required interface for
// access to the graph from the side of graph algorithms
template<class n_t>
class CBaseNodeIterator
{
    using self_t=CBaseNodeIterator<n_t>;
    using weight_t=typename n_t::weight_t;
    using node_t=n_t;
    using edge_t=typename node_t::edge_t;
    using adj_iterator=typename node_t::const_adj_iterator;

    node_t* m_nodefrom=nullptr;
    adj_iterator  m_adj_iterator;
    // this constructor can be called only
    // from node_t::get_iterator_to
    explicit CBaseNodeIterator(node_t*from,adj_iterator ai):
    m_nodefrom(from),m_adj_iterator(ai)
    {
    }
    public:
    CBaseNodeIterator& operator=(const CBaseNodeIterator<n_t>&)=default;
    CBaseNodeIterator(const CBaseNodeIterator<n_t>&)=default;
    CBaseNodeIterator(){}
    explicit CBaseNodeIterator(const node_t& nodefrom)
    {
        m_nodefrom=const_cast<node_t*>(&nodefrom);
        m_adj_iterator=nodefrom.m_adjacents.begin();
    }

    void set_base(const node_t& node)
    {
        m_nodefrom=const_cast<node_t*>(&node);
        m_adj_iterator=m_nodefrom->m_adjacents.begin();
    }

    void next(){++m_adj_iterator;}

    node_t& to_node()const
    {
        return *(m_adj_iterator->node);
    }
    node_t& from_node()const
    {
        return *m_nodefrom;
    }
    edge_t& edge()const
    {
        return *(m_adj_iterator->edge.get());
    }

    // reverse edge - same as edge() for undirected edge,
    // nullptr for directed, !=  for 2- way
    edge_t* get_reverse_edge()const
    {
        return to_node().get_edge_to(from_node());
    }
    // iterator for reverse edge
    std::optional<CBaseNodeIterator> get_reverse_iterator()const
    {
        return to_node().get_iterator_to(from_node());
    }
    int type()const
    {
        edge_t* reverse_edge=get_reverse_edge();
        if(reverse_edge==nullptr) return id_1_2_edge;
        if(reverse_edge==&edge()) return id_undirect_edge;
        return id_bidirect_edge;
    }
    void begin(){m_adj_iterator=m_nodefrom->m_adjacents.begin();}
    bool end()const{return m_adj_iterator==m_nodefrom->m_adjacents.end();}
    weight_t weight()const
    {
        return edge().weight();
    }
    template<class  ed_t,class derived_node_t >
    friend std::optional<CBaseNodeIterator<derived_node_t>>
    CBaseNode<ed_t,derived_node_t>::get_iterator_to(const derived_node_t&)const;
};

//get_iterator_to
// Complexity - logarithmic of the number of outgoing edges "this"

template<class  ed_t,class derived_node_t >
std::optional<CBaseNodeIterator<derived_node_t>>
CBaseNode<ed_t,derived_node_t>::get_iterator_to(const derived_node_t& other)const
{
    auto adj_iter=std::lower_bound(m_adjacents.begin(),
                                   m_adjacents.end(),
                                   other,_adj_comp_node());

    if(adj_iter!=m_adjacents.end()&&adj_iter->node==&other)
    {
        return iterator{const_cast<node_t*>(static_cast<const node_t*>(this)),adj_iter};
    }
    else
    {
        return {};
    }
}

template<class node_t>
TypeEdge _aux_edge_type(const typename node_t::edge_t*edge_1_2,
                   const typename node_t::edge_t*edge_2_1)
{
    if(edge_1_2!=nullptr)
    {
        if(edge_2_1!=nullptr)
        {
            if(edge_1_2==edge_2_1)
            {
                return id_undirect_edge;
            }
            else
            {
                return id_bidirect_edge;
            }
        }
        else
        {
            return id_1_2_edge;
        }
    }
    else
    {
        if(edge_2_1!=nullptr)
        {
            return id_2_1_edge;
        }
        else
        {
            return id_no_edge;
        }
    }
}

template<class node_t>
TypeEdge edge_type(const node_t&from,const node_t&_to)
{
    return _aux_edge_type<node_t>(from.get_edge_to(_to),
                          _to.get_edge_to(from));
}


// CGraphComposer
// Responsible for building and changing the graph, template node_t
// must provide CBaseNode interface

template<class node_t>
class CGraphComposer
{
   std::vector<node_t*> m_nodes;
   using edge_t=typename node_t::edge_t;
   int m_num_direct_edges;
   int m_num_undirect_edges;
   int m_num_bidirect_edges;
   public:
   CGraphComposer(const CGraphComposer&)=delete;
   CGraphComposer& operator=(const CGraphComposer&)=delete;
   CGraphComposer():
   m_num_direct_edges(0),
   m_num_undirect_edges(0),
   m_num_bidirect_edges(0)
   {}
   CGraphComposer(int  num_nodes):CGraphComposer()
   {
       create_nodes( num_nodes);
   }
   void create_nodes(int  num_nodes)
   {
       for(int i=0;i<num_nodes;++i)
       {
           add_node(new node_t);
       }
   }
   int add_node(node_t* new_node )
   {
       m_nodes.push_back(new_node);
       new_node->m_index=m_nodes.size()-1;
       return new_node->m_index;
   }
   //////////////////////////////////////////////////////////////////
   // Creation and destruction of connections between nodes.      ///
   // The end result does NOT depend on the previous edge state   ///
   // return- the type of the previous link.                       //
   //////////////////////////////////////////////////////////////////
   // undirected edge
   TypeEdge set_undirect_edge(node_t& fst,node_t& snd,edge_t*ed_ptr)
   {
       TypeEdge prev=delete_edge(fst,snd);
       std::shared_ptr<edge_t> edge(ed_ptr);
       fst.set_edge(snd,edge);
       snd.set_edge(fst,edge);
       m_num_undirect_edges++;
       return prev;
   }
   // direceted edge from fst to snd
   TypeEdge set_direct_edge(node_t& fst,node_t& snd,edge_t*ed_ptr)
   {
        TypeEdge prev=delete_edge(fst,snd);
        fst.set_edge(snd,std::shared_ptr<edge_t>(ed_ptr));
        snd.add_nonavailable(fst);
        m_num_direct_edges++;
        return prev;
   }
   // pair of directed edges, which connecte 2 nodes
   TypeEdge set_bidirect_edge(node_t& fst,node_t& snd,edge_t* edge_1_2,edge_t* edge_2_1)
   {
       TypeEdge prev=delete_edge(fst,snd);
       assert(edge_1_2!=edge_2_1);
       fst.set_edge(snd,std::shared_ptr<edge_t>(edge_1_2));
        snd.set_edge(fst,std::shared_ptr<edge_t>(edge_2_1));
       m_num_bidirect_edges++;
       return prev;
   }
   // edge of this type
   TypeEdge set_typed_edge(node_t& fst,node_t& snd,edge_t* edge_1_2,int type)
   {
       TypeEdge prev=delete_edge(fst,snd);
       switch(type)
       {
           case id_1_2_edge:set_direct_edge(fst,snd,edge_1_2);
           break;
           case id_2_1_edge:set_direct_edge(snd,fst,edge_1_2);
           break;
           case id_undirect_edge: set_undirect_edge(snd,fst,edge_1_2);
           break;
           default:assert(false);
       }
       return prev;
   }
   // destroy any edge between nodes
   TypeEdge delete_edge(node_t& fst,node_t& snd)
   {
       const edge_t* _1_2_tl=fst.delete_edge(snd);
       const edge_t* _2_1_tl=snd.delete_edge(fst);
       TypeEdge _type=_aux_edge_type<node_t>(_1_2_tl,_2_1_tl);
       switch(_type)
       {
            case id_undirect_edge:
            m_num_undirect_edges--;
            break;

            case id_bidirect_edge:
            m_num_bidirect_edges--;
            break;

            case id_1_2_edge:
            case id_2_1_edge:
            m_num_direct_edges--;
            break;

            case id_no_edge:break;
            default: assert(false);
       }

       return _type;
   }
   void delete_node(node_t& nd)
   {
       int _ind=nd.index();
       assert(m_nodes[_ind]==&nd);
       // destroy all edges nd
       while(!nd.m_adjacents.empty())
       {
           delete_edge(nd,*nd.m_adjacents.back().node);
       }
       while(!nd.m_adj_nonavailables.empty())
       {
           delete_edge(nd,*nd.m_adj_nonavailables.back());
       }
       // delete nd
       auto iter=std::find(m_nodes.begin(),m_nodes.end(),&nd);
       std::iter_swap(iter,m_nodes.end()-1);
       delete &nd; m_nodes.pop_back();

       if(!m_nodes.empty())
       {
           m_nodes[_ind]->m_index=_ind;
       }
   }
   void delete_all_edges()
   {
       for(node_t*nd:nodes())
       {
            while(!nd->m_adjacents.empty())
            {
                delete_edge(*nd,*(nd->m_adjacents.back().node));
            }
            while(!nd->m_adj_nonavailables.empty())
            {
                delete_edge(*nd,*(nd->m_adj_nonavailables.back()));
            }
       }
   }

   void delete_graph()
   {
       while(!m_nodes.empty())
       {
           delete_node(*m_nodes.front());
       }
   }

   const std::vector<node_t*>& nodes() const
   {
       return m_nodes;
   }

   int undirect_edges()const{return m_num_undirect_edges;}
   int direct_edges()const{return m_num_direct_edges;}
   int bidirect_edges()const{return m_num_bidirect_edges;}
   int all_edges()const{return m_num_undirect_edges+
                               m_num_direct_edges+
                               m_num_bidirect_edges;}
   ~CGraphComposer()
   {
   }
};

#endif




