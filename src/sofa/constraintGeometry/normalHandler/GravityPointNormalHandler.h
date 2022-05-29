#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa::constraintGeometry {

template<class DataTypes>
class GravityPointNormalHandler : public BaseNormalHandler {
public:

    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    SOFA_CLASS(SOFA_TEMPLATE(GravityPointNormalHandler,DataTypes), BaseNormalHandler);

    core::objectmodel::SingleLink<GravityPointGeometry<DataTypes>,GEOMETRY,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    void init() {

        for (unsigned i=0; i<l_geometry->getTopoProx().size(); i++)
        {
            l_geometry->getTopoProxIdx(i)->setNormal((l_geometry->getTopoProxIdx(i)->getPosition() - m_gcenter).normalized());
        }
    }

    void prepareDetection() override {
        m_gcenter = type::Vector3();

        const helper::ReadAccessor<DataVecCoord> & pos = this->l_geometry->getState()->read(core::VecCoordId::position());

        for (unsigned i=0;i<pos.size();i++) {
            m_gcenter += pos[i];
        }

        if (pos.size()) m_gcenter*=1.0/pos.size();

		init();
    }

private :
    type::Vector3 m_gcenter;
};


}
