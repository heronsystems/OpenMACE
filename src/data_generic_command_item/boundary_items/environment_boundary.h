#ifndef ENVIRONMENT_BOUNDARY_H
#define ENVIRONMENT_BOUNDARY_H

#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include "base/pose/geodetic_position_2D.h"



//Simple class to contain the values of the global environment boundary expected by the new GUI as of 09/2020

namespace BoundaryItem {

class EnvironmentBoundary
{
private:
   std::string m_name;
   std::string m_type;
   std::vector<mace::pose::GeodeticPosition_2D> m_vertices;

public:
   //!
   //! \brief Constructor for environment boundary
   //!
   //! Boundary is true for all vehicles
   //!
   EnvironmentBoundary() :
       m_name("noname"),
       m_type("hard")
   {
       m_vertices = {};
   }

public:
   //!
   //! \brief setValues Change only the vertices of the environment boundary
   //!
   //! \param vertices Vector of latlng points
   //!
   void setValues(const std::vector<mace::pose::GeodeticPosition_2D> &vertices) {
       m_vertices = vertices;
   }

   //!
   //! \brief setValues Change vertices, name and type of environment boundary
   //!
   //! \param vertices Vector of latlng points
   //! \param type Type of boundary ("hard" or "soft")
   //! \param name Boundary name
   //!
   void setValues(const std::vector<mace::pose::GeodeticPosition_2D> &vertices, const std::string &type, const std::string &name = "noname") {
       m_vertices = vertices;
       m_type = type;
       m_name = name;
   }

   //!
   //! \brief getVertices Returns the current vector of boundary vertices
   //!
   //! \return vertices Vector of latlng points
   //!
   std::vector<mace::pose::GeodeticPosition_2D> getVertices() {
       std::vector<mace::pose::GeodeticPosition_2D> vertices = m_vertices;
       return vertices;
   }

   //!
   //! \brief getName Returns the name of this environment boundary
   //!
   //! \return name Name of this environment boundary
   //!
   std::string getName() {
       std::string name = m_name;
       return name;
   }

   //!
   //! \brief getType Returns the type of this environment boundary
   //!
   //! \return type Type of this environment boundary ("hard" or "soft")
   //!
   std::string getType() {
       std::string type = m_type;
       return type;
   }

   EnvironmentBoundary& operator =(const EnvironmentBoundary &rhs)
   {
       this->m_vertices = rhs.m_vertices;
       this->m_name = rhs.m_name;
       this->m_type = rhs.m_type;
       return *this;
   }


};

} //end of namespace BoundaryItem


#endif // ENVIRONMENT_BOUNDARY_H
