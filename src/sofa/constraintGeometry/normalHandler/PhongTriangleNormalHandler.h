#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa::constraintGeometry {

template<class DataTypes>
class PhongTriangleNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(SOFA_TEMPLATE(PhongTriangleNormalHandler,DataTypes), BaseNormalHandler);

    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<PhongTriangleGeometry<DataTypes>,GEOMETRY,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    PhongTriangleGeometry()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

    type::Vector3 getNormal(collisionAlgorithm::BaseProximity::SPtr prox) override {
        if (collisionAlgorithm::TriangleProximity::SPtr tprox = std::dynamic_pointer_cast<collisionAlgorithm::TriangleProximity>(prox)) {
            auto element = tprox->element();


            for (auto it = element->pointElements().cbegin();it != element->pointElements().cend(); it++) {
                collisionAlgorithm::PointElement::SPtr pe = *it;

                type::Vector3 N;

                for (auto it2 = pe->triangleAround().cbegin();it2!=pe->triangleAround().cend();it2++) {
                    collisionAlgorithm::TriangleElement::SPtr te = *it2;

                    N+=te->getTriangleInfo().N;
                }


            }
        }

        std::cerr << "Error the proximity is no a TriangleProximity in GouraudTriangleNormalHandler " << std::endl;
        return type::Vector3();
    }

    void prepareDetection() override {

        for (unsigned i=0;i<l_geometry->getTopoProx().size();i++) {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->l_geometry->l_topology->getTrianglesAroundVertex(i);
            type::Vector3 N(0,0,0);
            for (size_t t=0;t<tav.size();t++) {
                unsigned eid = tav[t];
                auto element = l_geometry->getElements()[eid];
                N += element->getTriangleInfo().N;
            }
            N.normalize();

            l_geometry->getTopoProx()[i]->setNormal(N);
        }
    }

};



}
