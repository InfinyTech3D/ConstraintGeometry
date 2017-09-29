#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_H

#include <math.h>
#include <sofa/defaulttype/Vec.h>
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

class BaseGeometry;

class ConstraintProximity {
    friend class BaseGeometry;
    friend class CollisionAlgorithm;
    friend class ConstraintNormal;

public :
    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

    ConstraintProximity(const BaseGeometry * geo) {
        m_geo = geo;
    }

    virtual defaulttype::Vector3 getPosition() const = 0;

    virtual defaulttype::Vector3 getFreePosition() const = 0;

    virtual void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) = 0;

    virtual void getControlPoints(helper::vector<defaulttype::Vector3> & controlPoints) = 0;

    virtual defaulttype::Vector3 getNormal() = 0;

    virtual void refineToClosestPoint(const Coord & P);

    bool operator ==(const ConstraintProximity & b) const {
        if (m_pid.size() != b.m_pid.size()) return false;

        for (unsigned i=0;i<m_pid.size();i++) {
            bool find = false;
            unsigned j=0;
            while (j<m_pid.size() && !find) {
                find = m_pid[i] == b.m_pid[j] && m_fact[i] == b.m_fact[j];
                j++;
            }
            if (!find) return false;
        }

        return true;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const ConstraintProximity& c ) {
        for (unsigned i=0;i<c.m_fact.size();i++) {
            out << "[" << c.m_pid[i] << "," << c.m_fact[i] << "]";
        }
        return out;
    }

    const helper::vector<unsigned> & getPid() {
        return m_pid;
    }

    const helper::vector<double> & getFact() {
        return m_fact;
    }

protected:
    const BaseGeometry * m_geo;
    helper::vector<unsigned> m_pid;
    helper::vector<double> m_fact;

private:

    virtual void inc(const helper::vector<double> & dir);
};

typedef std::shared_ptr<ConstraintProximity> ConstraintProximityPtr;

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
