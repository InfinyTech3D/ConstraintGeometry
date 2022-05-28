#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class GouraudTriangleGeometry : public CollisionComponent {
public:

    SOFA_CLASS(SOFA_TEMPLATE(GouraudTriangleGeometry,DataTypes), CollisionComponent);

    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<GouraudTriangleGeometry<DataTypes>,GEOMETRY,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    GouraudTriangleGeometry()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

    class GouraudTriangleProximity : public TriangleProximity {
    public:
        typedef sofa::core::behavior::MechanicalState<DataTypes> State;

        GouraudTriangleProximity(BaseProximity::SPtr p0,BaseProximity::SPtr p1, BaseProximity::SPtr p2, double f0,double f1,double f2,const type::Vector3 & N)
        : TriangleProximity(p0,p1,p2,f0,f1,f2)
        , m_N(N) {}

        virtual sofa::type::Vector3 getNormal() const override { return m_N; }

    private:
        const type::Vector3 & m_N;
    };

    void init() {
        l_geometry->setCreateTriangleProximity(
            [=](const TriangleElement * elmt, double f0,double f1,double f2) -> BaseProximity::SPtr {
                return BaseProximity::create<GouraudTriangleProximity>(elmt->getP0(),elmt->getP1(),elmt->getP2(),
                                                                      f0,f1,f2,
                                                                      elmt->getTriangleInfo().N);
            }
        );
    }

    void prepareDetection() override {}

};

}
