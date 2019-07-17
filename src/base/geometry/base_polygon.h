#ifndef BASE_POLYGON_H
#define BASE_POLYGON_H

#include <vector>
#include <stdlib.h>

#include "geometry_helper.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

namespace mace{
namespace geometry{

class PolygonAbstract
{
    public:
    PolygonAbstract(const std::string &descriptor = "Polygon"):
        name(descriptor)
    {

    }

    virtual ~PolygonAbstract() = default;

    template <class T>
    T *as()
    {
        return static_cast<T*>(this);
    }

    template <class T>
    const T *as() const
    {
        return static_cast<const T*>(this);
    }

    PolygonAbstract(const PolygonAbstract &copy)
    {
        this->name = copy.name;
    }

    virtual mace::pose::CoordinateFrame getVertexCoordinateFrame() const = 0;

public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    PolygonAbstract& operator = (const PolygonAbstract &rhs)
    {
        this->name = rhs.name;
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const PolygonAbstract &rhs) const
    {
        if(this->name != rhs.name)
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const PolygonAbstract &rhs) const {
        return !(*this == rhs);
    }

    protected:
        std::string name;
};

template <class T>
class PolygonBase : PolygonAbstract
{
public:
    PolygonBase(const std::string &descriptor = "Polygon"):
        PolygonAbstract(descriptor)
    {

    }

    PolygonBase(const std::vector<T> &vector, const std::string &descriptor = "Polygon"):
        PolygonAbstract(descriptor)
    {
        //this->clearPolygon(); we should not have to call this case since this is in the constructer
        m_vertex = vector;
        updateBoundingBox();
    }

    PolygonBase(const PolygonBase &copy):
        PolygonAbstract(copy)
    {
        this->replaceVector(copy.m_vertex);
    }

    bool isValidPolygon() const
    {
        return (m_vertex.size() >= 3);
    }

    void initializePolygon(const unsigned int &size)
    {
        if(size <= 0){
            std::cout << "Cannot initialize a polygon of size 0." << std::endl;
            throw std::exception();
        }
        m_vertex.clear();
        std::vector<T> tmpVector(size,T());
        m_vertex = tmpVector;
        updateBoundingBox();
    }

    void appendVertex(const T &vertex)
    {
        m_vertex.push_back(vertex);
        updateBoundingBox();
    }

    void insertVertexAtIndex(const T &vertex, const unsigned int &index)
    {
        m_vertex[index] = vertex;
        updateBoundingBox();
    }

    void removeVertex(const int &index);

    void replaceVector(const std::vector<T> &vector)
    {
        this->clearPolygon();
        m_vertex = vector;
        updateBoundingBox();
    }

    void clearPolygon()
    {
        m_vertex.clear();
        m_vertex.shrink_to_fit();
    }

    size_t polygonSize() const
    {
        return m_vertex.size();
    }

    //!
    //! \brief getVector
    //! \return
    //!
    std::vector<T> getVector() const
    {
        return m_vertex;
    }

    T getVertexAtIndex(const unsigned int &index) const
    {
        if(index < m_vertex.size())
            return nullptr;
        else
            m_vertex.at(index);
    }

    T at(const int &index) const
    {
        return m_vertex[index];
    }

public:
    virtual T getTopLeft() const = 0;

    virtual T getTopRight() const = 0;

    virtual T getBottomLeft() const = 0;

    virtual T getBottomRight() const = 0;

    virtual void getCorners(T &topLeft, T &bottomRight) const = 0;

    virtual std::vector<int> findUndefinedVertices() const = 0;

    /** Assignment Operators **/
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    PolygonBase& operator = (const PolygonBase &rhs)
    {
        PolygonAbstract::operator =(rhs);
        this->m_vertex = rhs.m_vertex;
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const PolygonBase &rhs) const
    {
        if(!PolygonAbstract::operator ==(rhs))
        {
            return false;
        }
        if(this->m_vertex.size() != rhs.m_vertex.size())
        {
            return false;
        }
        for(unsigned int i = 0; i < this->m_vertex.size(); i++)
        {
            if(m_vertex.at(i) != rhs.m_vertex.at(i))
                return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const PolygonBase &rhs) const {
        return !(*this == rhs);
    }


protected:
    virtual void updateBoundingBox()
    {

    }

protected:
    std::vector<T> m_vertex;
};

} //end of namepsace geometry
} //end of namespace mace
#endif // BASE_POLYGON_H
