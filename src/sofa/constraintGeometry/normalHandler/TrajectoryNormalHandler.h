#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa::constraintGeometry {

template<class DataTypes>
class TrajectoryNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(SOFA_TEMPLATE(TrajectoryNormalHandler,DataTypes), BaseNormalHandler);

    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<TrajectoryGeometry<DataTypes>,EdgeGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    TrajectoryGeometry()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

    void prepareDetection() override {

      for (unsigned i=0;i<l_geometry->getTopoProx().size();i++)
      {
        type::Vec3d Tang;
        if(i == 0)
        {
          Tang = l_geometry->getTopoProx()[i+1]->getPosition()-l_geometry->getTopoProx()[i]->getPosition();
          Tang.normalize();
        }
        else if (i == l_geometry->getTopoProx().size()-1)
        {
          Tang = l_geometry->getTopoProx()[i]->getPosition()-l_geometry->getTopoProx()[i-1]->getPosition();
          Tang.normalize();
        }
        else
        {
          type::Vec3d T1 =  l_geometry->getTopoProx()[i+1]->getPosition()-l_geometry->getTopoProx()[i]->getPosition();
          T1.normalize();
          type::Vec3d T2 =  l_geometry->getTopoProx()[i]->getPosition()-l_geometry->getTopoProx()[i-1]->getPosition();
          T2.normalize();

          Tang = (T1 + T2)/2;
          Tang.normalize();
        }
        l_geometry->getTopoProx()[i]->setNormal(Tang);
      }
    }

};

}
