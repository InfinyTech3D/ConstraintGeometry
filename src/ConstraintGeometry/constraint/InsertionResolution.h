#pragma once

#include <ConstraintGeometry/BaseConstraint.h>
#include <ConstraintGeometry/directions/BindDirection.h>
#include <sofa/core/behavior/ConstraintResolution.h>
#include <sofa/helper/logging/Messaging.h>
#include <cmath>

namespace sofa {

namespace constraintgeometry {

class InsertionConstraintResolution : public sofa::core::behavior::ConstraintResolution {
public:
    InsertionConstraintResolution(SReal frictionCoeff, double maxf1 = std::numeric_limits<double>::max(),double maxf2 = std::numeric_limits<double>::max(),double maxf3 = std::numeric_limits<double>::max(),
                                   double comp1 = 0.0,double comp2 = 0.0,double comp3 = 0.0,
                                   bool scaleComplianceMatrix = false) : sofa::core::behavior::ConstraintResolution(3) {
        m_maxForce[0] = maxf1;
        m_maxForce[1] = maxf2;
        m_maxForce[2] = maxf3;

        m_compliance[0] = comp1;
        m_compliance[1] = comp2;
        m_compliance[2] = comp3;

        m_frictionCoeff = frictionCoeff;
        m_scaleComplianceMatrix = scaleComplianceMatrix;
    }

    virtual void init(int line, double** w, double * /*force*/)
    {
        sofa::type::Mat<3,3,double> temp;
        for (unsigned j=0;j<3;j++) {
            for (unsigned i=0;i<3;i++) {
                temp[j][i] = w[line+j][line+i];

                if (i == j) temp[j][i]+=m_compliance[i];
            }
        }

        bool inverted = sofa::type::invertMatrix(invW, temp);

        // invertMatrix rejects the inversion if |det| < machine-eps, leaves invW empty and returns 
        // false. However, |det| depends on the problem scale and the physical parameters. 
        // It can happen that a matrix is well-conditioned but with a small |det|. In that case, 
        // proceed by scaling the matrix before inversion, then scale it back up afterwards.
        if (!inverted && m_scaleComplianceMatrix)
        {
            const double s = (std::abs(temp[0][0]) + std::abs(temp[1][1]) + std::abs(temp[2][2])) / 3.0;
            if (s > 0.0)
            {
                const double invS = 1.0 / s;
                inverted = sofa::type::invertMatrix(invW, temp * invS);
                invW *= invS;
            }
        }

        // Matrix is rank-deficient. Nothing could be done.
        if (!inverted)
            msg_warning("InsertionConstraintResolution")
                << "Failed to invert the compliance matrix of the constraint at line " << line << ". If not already done, try to scale compliance matrix using Data scaleComplianceMatrix";
    }

    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/)
    {
        std::array<SReal, 3> corr = { 0.0, 0.0, 0.0 };

        for (int j = 0; j < 3; j++)
            corr[0] -= invW[0][j] * d[line+j];
        corr[0] *= m_frictionCoeff;

        // Back solve 2x2 system with friction on the RHS
        const SReal denom = w[line+1][line+1] * w[line+2][line+2] - w[line+1][line+2] * w[line+2][line+1];
        const SReal nom1 = w[line+1][line+2] * (d[line+2] + w[line+2][line+0] * corr[0]);
        const SReal nom2 = w[line+2][line+2] * (d[line+1] + w[line+1][line+0] * corr[0]);
        corr[1] = (nom1 - nom2) / denom;

        corr[2] = -(d[line+2] + w[line+2][line+0] * corr[0] + w[line+2][line+1] * corr[1]) / w[line+2][line+2];

        for (int i = 0; i < 3; i++) 
            force[line+i] += corr[i];

        for(int i = 0; i < 3; i++) {
            if (force[line+i] > m_maxForce[i]) force[line+i]=m_maxForce[i];
            else if (force[line+i] < -m_maxForce[i]) force[line+i]=-m_maxForce[i];
        }

        //SReal slip_force = m_frictionCoeff;
        //if (force[line+2] >  slip_force) force[line+2] = slip_force;
        //else if (force[line+2] < -slip_force) force[line+2] = -slip_force;
    }

    double m_maxForce[3];
    double m_compliance[3];
    sofa::type::Mat<3,3,double> invW;
    SReal m_frictionCoeff;
    bool m_scaleComplianceMatrix;
};

}

}
