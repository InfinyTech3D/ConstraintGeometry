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
#include <sofa/helper/AdvancedTimer.h>

namespace sofa {

namespace core {

namespace behavior {

class ConstraintProximity;
class CollisionAlgorithm;

class BaseConstraintGeometry : public core::BehaviorModel {
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
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    //    topology::TopologyContainer * getTopology() {
    //        return dynamic_cast<topology::TopologyContainer *>(getContext()->getTopology());
    //    }
    topology::BaseMeshTopology * getTopology() {
        return dynamic_cast<topology::BaseMeshTopology *>(getContext()->getTopology());
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getMstate() {
        return dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes> * >(getContext()->getMechanicalState());
    }

    virtual void prepareDetection() {}

private :
    void updatePosition(SReal /*dt*/) {
        std::string timerName = std::string("updatePosition")+this->getName();

        sofa::helper::AdvancedTimer::stepBegin(timerName.c_str());
        prepareDetection();
        sofa::helper::AdvancedTimer::stepEnd(timerName.c_str());
    }

};

class BaseDecorator : public BaseConstraintGeometry {

};


class BaseGeometry : public BaseConstraintGeometry {
public :
    SOFA_CLASS(BaseGeometry, BaseConstraintGeometry);

    virtual void init();

    virtual void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const defaulttype::Vector3 & normal);

    virtual defaulttype::Vector3 getPosition(const ConstraintProximity & pinfo);

    virtual defaulttype::Vector3 getFreePosition(const ConstraintProximity & pinfo);

    virtual defaulttype::Vector3 getRestPosition(const ConstraintProximity & pinfo);

    virtual defaulttype::Vector3 getNormal(const ConstraintProximity & pinfo) = 0;

    virtual double projectPoint(unsigned eid,const defaulttype::Vector3 & T, ConstraintProximity & pinfo) = 0;

    virtual int getNbElements() = 0;

    virtual BaseDecorator * getDecorator();

    //    void getAlgorithm(CollisionAlgorithm * algo);

    //    void createAlgorithm(CollisionAlgorithm * alg);
};

//class CollisionAlgorithm : public BaseConstraintGeometry {
//public:

//    BaseGeometry * getGeometry();

//};


} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
