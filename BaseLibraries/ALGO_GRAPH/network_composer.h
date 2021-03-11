
#ifndef  _network_composer_
#define  _network_composer_
#include <assert.h>
#include <vector>
#include <algorithm>
#include <utility>


template<class flow_t>
class network_node_iterator_t;

template<class flow_t>
class network_t;

template<class flow_t>
class network_node_t
{
    public:
    class edge_t
    {
        bool m_in_list=false;
        const flow_t m_max_capacity;
        flow_t m_residual_capacity;
        edge_t* m_prev=nullptr;
        edge_t* m_next=nullptr;

        network_node_t&m_sourse;
        edge_t* m_reverse_edge=nullptr;

        public:
        edge_t(flow_t max_capacity,network_node_t&source):
        m_max_capacity(max_capacity),
        m_residual_capacity(max_capacity),
        m_sourse(source)
        {}
        network_node_t& source(){return m_sourse;}
        network_node_t& adjacent_node()const{return m_reverse_edge->m_sourse;}
        bool is_in_list()const{return m_in_list;}
        const edge_t* next()const{return m_next;}
        const edge_t* previous()const{return m_prev;}
        edge_t& reverse_edge()const{return *m_reverse_edge;}
        flow_t residual_capacity()const{return m_residual_capacity;}
        flow_t max_capacity()const{return m_max_capacity;}
        flow_t flow()const{return m_residual_capacity-m_max_capacity;}
        void insert_to_list(edge_t*prev,edge_t*next)
        {
            assert(!m_in_list);
            m_prev=prev; m_next=next;
            if(m_prev!=nullptr)
            {
                m_prev->m_next=this;
            }
            if(m_next!=nullptr)
            {
                m_next->m_prev=this;
            }
            m_in_list=true;
        }
        void erase_from_list()
        {
            assert(m_in_list);
            if(m_prev!=nullptr)
            {
                m_prev->m_next=m_next;
            }
            if(m_next!=nullptr)
            {
                m_next->m_prev=m_prev;
            }
            m_prev=m_next=nullptr;
            m_in_list=false; //2 hours
        }
        void add_flow(flow_t flow)
        {
            assert(m_in_list);

            m_residual_capacity-=flow;
            assert(m_residual_capacity>=0);
            m_reverse_edge->m_residual_capacity+=flow;
            /*if(!m_reverse_edge->m_in_list)
            {
                adjacent_node().insert(*m_reverse_edge);
            }*/
        }

        void fill()
        {
            assert(m_in_list);

            m_residual_capacity=zero_flow();
            m_reverse_edge->m_residual_capacity=
            m_max_capacity+m_reverse_edge->m_max_capacity;
            /*if(!m_reverse_edge->m_in_list)
            {
                adjacent_node().insert(*m_reverse_edge);
            }
            m_sourse.erase(edge);*/
        }
        void to_empty()
        {
            m_residual_capacity=m_max_capacity;
            m_reverse_edge->m_residual_capacity=
            m_reverse_edge->m_max_capacity;
        }
        void erase(){m_sourse.erase(*this);}
        void insert(){m_sourse.insert(*this);}

        static void set_pipe(network_node_t<flow_t>& n1,
                             network_node_t<flow_t>& n2,
                             flow_t cap1,flow_t cap2)
        {
            edge_t* edge1_2=new edge_t(cap1,n1);
            edge_t* edge2_1=new edge_t(cap2,n2);
            edge1_2->m_reverse_edge=edge2_1;
            edge2_1->m_reverse_edge=edge1_2;
            n1.add(*edge1_2);
            n2.add(*edge2_1);
        }
    };
    using weight_t=flow_t; // adapte to priority search
    private:
    std::vector<edge_t*> m_edges;
    edge_t* m_edges_front=nullptr;
    int m_index;
    public:
    using iterator=network_node_iterator_t<flow_t>;
    static flow_t zero_flow(){ return flow_t{0};}
    explicit network_node_t(int index):
    m_index(index)
    {}
    network_node_t(const network_node_t& node)=delete;
    network_node_t& operator=(const network_node_t& node)=delete;
    void erase(edge_t&edge)
    {
        if(edge.previous()==nullptr)
        {
            assert(&edge==m_edges_front);
            m_edges_front=const_cast<edge_t*>(edge.next());
        }
        edge.erase_from_list();
    }
    void insert(edge_t&edge)
    {
        edge.insert_to_list(nullptr,m_edges_front);
        m_edges_front=&edge;
    }
    void add(edge_t&edge)
    {
        m_edges.push_back(&edge);
        insert(edge);
    }
    const edge_t* get_edge_to(const network_node_t&other)const
    {
        auto res=std::find_if(m_edges.begin(),m_edges.end(),
                              [&other](const edge_t*edge)
                              {
                                  return &edge->adjacent_node()==&other;
                              });
        return (res==m_edges.end())? nullptr:*res;
    }
    int index()const{return m_index;}
    ~network_node_t()
    {
        for(auto edge_ptr:m_edges){delete edge_ptr;}
    }

    friend class network_t<flow_t>;
    friend class network_node_iterator_t<flow_t>;
};


template<class flow_t>
class network_node_iterator_t
{
    using node_t=network_node_t<flow_t>;
    using edge_t=node_t::edge_t;
    node_t* m_source=nullptr;
    edge_t* m_current=nullptr;
    public:
    network_node_iterator_t(){}
    explicit network_node_iterator_t(const node_t&node)
    {
        m_source=const_cast<node_t*>(&node);
        m_current=m_source->m_edges_front;
    }
    void set_base(const node_t& node)
    {
        m_source=const_cast<node_t*>(&node);
        m_current=m_source->m_edges_front;
    }
    void next(){m_current=const_cast<edge_t*>(m_current->next());}
    flow_t capacity()const{return m_current->residual_capacity();}
    flow_t max_capacity()const{return m_current->max_capacity();}
    void add_flow(flow_t flow){m_current->add_flow(flow);}
    void fill(){m_current->fill();}

    node_t& to_node()const
    {
        return m_current->adjacent_node();
    }
    node_t& from_node()const
    {
        return *m_source;
    }
    edge_t& edge()const
    {
        return *m_current;
    }
    edge_t* get_reverse_edge()const
    {
        return &m_current->reverse_edge();
    }
    void begin(){m_current=m_source->m_edges_front;}
    bool end()const{return m_current==nullptr;}
};

template<class flow_t>
class network_t
{
    public:
    using node_t=network_node_t<flow_t>;
    using edge_t=node_t::edge_t;
    private:
    std::vector<node_t*> m_nodes;


    public:
    void destroy()
    {
        for(node_t* node:m_nodes) {delete node;}
        m_nodes.clear();
    }
    network_t(const network_t&)=delete;
    network_t& operator=(const network_t&)=delete;
    network_t()
    {}
    template<class input_iterator_t>
    void build(input_iterator_t begin,input_iterator_t end,std::size_t num_nodes)
    {
        destroy();
        for(std::size_t i=0;i<num_nodes;++i)
        {
            m_nodes.push_back(new node_t(i));
        }
        for(;begin!=end;++begin)
        {
            auto [from,to,cap_1_2,cap_2_1]=*begin;
            edge_t::set_pipe(*m_nodes[from],*m_nodes[to],cap_1_2,cap_2_1);
        }
        assert(check_adjacency());
    }
    flow_t flow(int ifrom,int ito)const
    {
        const node_t*from=m_nodes[ifrom];
        const node_t*to=m_nodes[ito];
        if(auto edge=from->get_edge_to(*to);edge!=nullptr)
        {
            return edge->flow();
        }
        else
        {
            return flow_t(0);
        }
    }
    std::pair<flow_t,flow_t> residual_capacities(int ifrom,int ito)const
    {
        const node_t*from=m_nodes[ifrom];
        const node_t*to=m_nodes[ito];
        if(auto edge=from->get_edge_to(*to);edge!=nullptr)
        {
            return {edge->residual_capacity(),
                    edge->reverse_edge().residual_capacity()};
        }
        else
        {
            return {flow_t(0),flow_t(0)};
        }
    }
    bool check_adjacency()const
    {
        std::vector<int> adj;
        for(auto node:m_nodes)
        {
            for(auto edge:node->m_edges)
            {
                adj.push_back(edge->adjacent_node().index());
            }
            std::sort(adj.begin(),adj.end());
            if(std::adjacent_find(adj.begin(),adj.end())!=adj.end()){return false;}
            adj.clear();
        }
        return true;
    }
    void to_empty()
    {
        auto update_adj_list=[](edge_t&edge)
        {
            if(!edge.is_in_list())
            {
                edge.insert();
            }
        };
        for(node_t* from: m_nodes)
        {
            for(edge_t* edge:from->m_edges)
            {
                // every pair of adjacent nodes is processed once
                if(from->index()>edge->adjacent_node().index())continue;

                edge->to_empty();
                update_adj_list(*edge);
                update_adj_list(edge->reverse_edge());
            }
        }
    }
    template<class visitor_t>
    void inspect(visitor_t visitor)const
    {
        for(node_t* from: m_nodes)
        {
            for(edge_t* edge:from->m_edges)
            {
                // every pair of adjacent nodes is processed once
                if(from->index()>edge->adjacent_node().index())continue;

                visitor(from->index(),
                        edge->adjacent_node().index(),
                        edge->residual_capacity(),
                        edge->reverse_edge().residual_capacity());
            }
        }
    }
    const std::vector<node_t*>& nodes(){ return m_nodes;}
};

#endif

