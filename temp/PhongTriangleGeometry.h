#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class PhongTriangleGeometry : public CollisionComponent {
public:

    SOFA_CLASS(SOFA_TEMPLATE(PhongTriangleGeometry,DataTypes), CollisionComponent);

    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<PhongTriangleGeometry<DataTypes>,GEOMETRY,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    PhongTriangleGeometry()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

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
