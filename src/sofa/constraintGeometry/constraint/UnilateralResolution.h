#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/directions/ContactDirection.h>
#include <sofa/core/behavior/ConstraintResolution.h>


namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The UnilateralConstraintResolution class solves unilateral constraints
 */
class UnilateralConstraintResolution : public sofa::core::behavior::ConstraintResolution {
public:

    /*!
     * \brief UnilateralConstraintResolution Constructor
     * \param m : double, maxForce value
     */
    UnilateralConstraintResolution(double m = std::numeric_limits<double>::max()) :
        sofa::core::behavior::ConstraintResolution(1),
        m_maxForce(m)
    {}

    /*!
     * \brief resolution : updates and keeps the force applied on 'line'
     * in the interval [0, maxForce]
     * \param line
     * \param w
     * \param d
     * \param force
     */
    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/) {
        force[line] -= d[line] / w[line][line];

        if (force[line]>m_maxForce)
            force[line] = m_maxForce;
        else if (force[line]<0)
            force[line] = 0.0;
    }

    double m_maxForce;
};

/*!
 * \brief The UnilateralFrictionResolution class
 * Solves unilateral friction constraints
 */
class UnilateralFrictionResolution : public sofa::core::behavior::ConstraintResolution {
public:
    UnilateralFrictionResolution(double f, double m0 = std::numeric_limits<double>::max(), double m1 = std::numeric_limits<double>::max(), double m2 = std::numeric_limits<double>::max()) :
        sofa::core::behavior::ConstraintResolution(3),
        m_maxForce0(m0),
        m_maxForce1(m1),
        m_maxForce2(m2),
        m_friction(f) {}


    virtual void init(int line, double** w, double * /*force*/) {
        sofa::type::Mat<3,3,double> temp;
        for (unsigned j=0;j<3;j++) {
            for (unsigned i=0;i<3;i++) {
                temp[j][i] = w[line+j][line+i];
            }
        }

        invertMatrix(invW, temp);
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/) {
        //we solve the first constraint as if there was no friction
        force[line+0] -= invW[0][0] * d[line+0] +
                         invW[0][1] * d[line+1] +
                         invW[0][2] * d[line+2] ;

        //we check unilateral conditions
        if (force[line]<0) {
            force[line+0] = 0.0;
            force[line+1] = 0.0;
            force[line+2] = 0.0;
            return;
        }

        //we clamp the force is needed
        if (force[line]>m_maxForce0) force[line] = m_maxForce0;

        //now we compute tangential forces
        force[line+1] -= invW[1][0] * d[line+0] +
                         invW[1][1] * d[line+1] +
                         invW[1][2] * d[line+2] ;

        force[line+2] -= invW[2][0] * d[line+0] +
                         invW[2][1] * d[line+1] +
                         invW[2][2] * d[line+2] ;


        double slip_force = force[line+0] * m_friction;

        if (force[line+1] >  slip_force) force[line+1] = slip_force;
        else if (force[line+1] < -slip_force) force[line+1] = -slip_force;

        if (force[line+2] >  slip_force) force[line+2] = slip_force;
        else if (force[line+2] < -slip_force) force[line+2] = -slip_force;

        if (force[line+1]>m_maxForce1) force[line+1] = m_maxForce1;
        if (force[line+2]>m_maxForce2) force[line+2] = m_maxForce2;
    }

    double m_maxForce0,m_maxForce1,m_maxForce2;
    double m_friction;
    sofa::type::Mat<3,3,double> invW;
};

}

}
