#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTRESPONSE_INL
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTRESPONSE_INL

#include "ConstraintResponse.h"
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

class ResponseGeometryB : public core::behavior::ConstraintResolution {
public:
    class ConstraintResolutionB : public core::behavior::ConstraintResolution {
    public:
        ConstraintResolutionB(double m) {
            maxForce = m;
        }

        virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/)
        {
            force[line] -= d[line] / w[line][line];

            if (force[line]>maxForce) force[line] = maxForce;
            else if (force[line]<-maxForce) force[line] = -maxForce;
        }

        double maxForce;

    };

    Data<double> d_maxf;

    ResponseGeometryB(core::objectmodel::BaseObject * o)
    : d_maxf(o->initData(&d_maxf, std::numeric_limits<double>::max(), "maxForce", "maxForce"))
    {}

    core::behavior::ConstraintResolution * getResolution() {
        return new ConstraintResolutionB(d_maxf.getValue());
    }

    static std::string Name() {
        return "B";
    }

    static ConstraintNormal makeConstraint(CollisionAlgorithm::PariProximity & pair) {
        defaulttype::Vector3 P = pair.first->getPosition();
        defaulttype::Vector3 Q = pair.second->getPosition();

        ConstraintNormal cn(pair.first,pair.second);
        double dist = (P-Q).norm();

        if (dist<0.0001) cn.setNormal(pair.second->getNormal());
        else cn.setNormal(P-Q);

        cn.orthogonalize(1);

        return cn;
    }

};

class ResponseGeometryU {
public:

    class ConstraintResolutionU : public core::behavior::ConstraintResolution {
    public:
        ConstraintResolutionU(double m) {
            maxForce = m;
        }

        virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/)
        {
            force[line] -= d[line] / w[line][line];

            if (force[line]*w[0][0]>maxForce) force[line] = maxForce;
            else if (force[line]*w[0][0]<0) force[line] = 0.0;
        }

        double maxForce;

    };

    Data<double> d_maxf;

    ResponseGeometryU(core::objectmodel::BaseObject * o)
    : d_maxf(o->initData(&d_maxf, std::numeric_limits<double>::max(), "maxForce", "maxForce"))
    {}

    core::behavior::ConstraintResolution * getResolution() {
        return new ConstraintResolutionU(d_maxf.getValue());
    }

    static std::string Name() {
        return "U";
    }

    static ConstraintNormal makeConstraint(CollisionAlgorithm::PariProximity & pair) {
        defaulttype::Vector3 P = pair.first->getPosition();
        defaulttype::Vector3 Q = pair.second->getPosition();

        defaulttype::Vector3 N = pair.second->getNormal();

        ConstraintNormal cn(pair.first,pair.second);

        if (dot(P-Q,N)<0) cn.setNormal(N);
        else cn.setNormal(P-Q);

        cn.orthogonalize(1);

        return cn;
    }
};

//class ResponseGeometryUFF {
//public:
//    static std::string Name() {
//        return "UFF";
//    }
//};

//class ResponseGeometryBBB {
//public:
//    static std::string Name() {
//        return "BBB";
//    }
//};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
