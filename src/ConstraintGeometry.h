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

typedef std::shared_ptr<ConstraintProximity> ConstraintProximityPtr;

class BaseConstraintIterator {
public:

    virtual bool end(const ConstraintProximityPtr & E = NULL) = 0;

    virtual int getElement() = 0;

    virtual void next() = 0;
};

typedef std::unique_ptr<BaseConstraintIterator> BaseConstraintIteratorPtr;

class BaseGeometry : public core::BehaviorModel {
    friend class ConstraintProximity;

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

    class DefaultConstraintIterator : public BaseConstraintIterator {
    public:
        DefaultConstraintIterator(const BaseGeometry * geo) {
            m_i = 0;
            m_geometry = geo;
        }

        virtual bool end(const ConstraintProximityPtr & /*E*/ = NULL) {
            return m_i>=m_geometry->getNbElements();
        }

        int getElement() {
            return m_i;
        }

        void next() {
            m_i++;
        }

    private :
        const BaseGeometry * m_geometry;
        int m_i;
    };

    Data<defaulttype::Vec4f> d_color;

    topology::TopologyContainer * getTopology() const {
        return m_topology;
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getMstate() const {
        return m_state;
    }

    BaseGeometry();

    void init();

    virtual ConstraintProximityPtr projectPoint(const defaulttype::Vector3 & T, unsigned eid) const = 0;

    virtual int getNbElements() const = 0;

    double getNorm() const;

    void computeBBox(const core::ExecParams* params, bool /*onlyVisible*/);

    BaseConstraintIteratorPtr getIterator(const ConstraintProximityPtr & E = NULL) const;

    virtual double getDistance(const ConstraintProximityPtr & T, const ConstraintProximityPtr & pinfo) const;

private :
    void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }

    sofa::core::behavior::MechanicalState<DataTypes> * m_state;
    topology::TopologyContainer * m_topology;

protected:

    virtual void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const defaulttype::Vector3 & normal) const;

    virtual void prepareDetection();

    defaulttype::Vector3 m_g;
    double m_norm;

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
