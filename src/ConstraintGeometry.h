#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_H
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/collision/Pipeline.h>
#include <SofaBaseTopology/PointSetTopologyContainer.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>

namespace sofa {

namespace core {

namespace behavior {

class ConstraintProximity;
class CollisionAlgorithm;
class BaseGeometry;

class BaseConstraintIterator {
public:

    virtual bool end(const ConstraintProximity & /*E*/) = 0;

    virtual int getElement() = 0;

    virtual void next() = 0;
};

class BaseDecorator : public core::BehaviorModel {
public:
    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;

    virtual std::unique_ptr<BaseConstraintIterator> getIterator(const ConstraintProximity & P) = 0;

    virtual void prepareDetection() {}

    BaseGeometry * getGeometry();

protected :
    void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }
};

class BaseGeometry : public core::BehaviorModel {
public :
    SOFA_CLASS(BaseGeometry, core::BehaviorModel);

    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    topology::TopologyContainer * getTopology() {
        return m_topology;
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getMstate() {
        return m_state;
    }

    class DefaultConstraintIterator : public BaseConstraintIterator {
    public:
        DefaultConstraintIterator(BaseGeometry * geo) {
            m_i = 0;
            m_geometry = geo;
        }

        virtual bool end(const ConstraintProximity & /*E*/) {
            return m_i>=m_geometry->getNbElements();
        }

        int getElement() {
            return m_i;
        }

        void next() {
            m_i++;
        }

    private :
        BaseGeometry * m_geometry;
        int m_i;
    };

    void init();

    virtual void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const defaulttype::Vector3 & normal);

    virtual defaulttype::Vector3 getPosition(const ConstraintProximity & pinfo);

    virtual defaulttype::Vector3 getFreePosition(const ConstraintProximity & pinfo);

    virtual defaulttype::Vector3 getRestPosition(const ConstraintProximity & pinfo);

    virtual defaulttype::Vector3 getNormal(const ConstraintProximity & pinfo) = 0;

    virtual double projectPoint(const defaulttype::Vector3 & T, ConstraintProximity & pinfo) = 0;

    virtual int getNbElements() = 0;

    std::unique_ptr<BaseConstraintIterator> getIterator(const ConstraintProximity & P);

    virtual void prepareDetection() {}

private :
    void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }

    sofa::core::behavior::MechanicalState<DataTypes> * m_state;
    topology::TopologyContainer * m_topology;

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
