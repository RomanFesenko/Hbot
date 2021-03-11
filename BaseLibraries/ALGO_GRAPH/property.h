
#ifndef  _property_
#define  _property_
#include <vector>


// Each object type Obj corresponds
// property type Pr
// Class Obj must have constant member-function index()

template <class Pr,class Obj>
class TProperty
{
    public:
    typedef Pr property_type;
    typedef Obj value_type;
    private:
    std::vector<property_type> m_properties;
    public:

    TProperty(const TProperty& other)=delete;
    TProperty& operator=(const TProperty& other)=delete;

    TProperty(int num_elems)
    {
        m_properties.resize(num_elems);
    }
    property_type operator()(const value_type& value) const
    {
        return m_properties[value.index()];
    }
    property_type& operator()(const value_type& value)
    {
        return m_properties[value.index()];
    }
    property_type* begin()
    {
        return &m_properties[0];
    }
    property_type* end()
    {
        return &m_properties.back()+1;
    }
    typename std::vector<property_type>::size_type
    size()const{return m_properties.size();}
};


#endif

