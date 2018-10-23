#pragma once

#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {


class UFFResponse : public BaseResponse {
public:
    SOFA_CLASS(UFFResponse, BaseResponse);

    Data<double> d_maxForce;
    Data<double> d_friction;

    UFFResponse()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "friction", "Friction")) {}

    class UnilateralConstraintResolution : public sofa::core::behavior::ConstraintResolution {
    public:
        UnilateralConstraintResolution(double m,double f) : ConstraintResolution(3), m_maxForce(m), m_friction(f) {}

        virtual void init(int line, double** w, double */*force*/) {
            sofa::defaulttype::Mat<3,3,double> temp;
            for (unsigned j=0;j<3;j++) {
                for (unsigned i=0;i<3;i++) {
                    temp[j][i] = w[line+j][line+i];
                }
            }

            invertMatrix(invW, temp);
        }

        virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/) {
            double bforce[3];
            for(int i=0; i<3; i++) bforce[line+i] = bforce[line+i];

            for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++)
                    force[line+i] -= d[line+j] * invW[i][j];
            }


            for(int i=0; i<3; i++) {
                if (force[line+i]>m_maxForce) force[line+i] = m_maxForce;
                else if (force[line+i]<0) force[line+i] = 0.0;
            }
        }

        double m_maxForce;
        double m_friction;
        sofa::defaulttype::Mat<3,3,double> invW;
    };

    virtual unsigned size() {
        return 3;
    }

    virtual core::behavior::ConstraintResolution* getConstraintResolution() {
        return new UnilateralConstraintResolution(d_maxForce.getValue(), d_friction.getValue());
    }

};

}

}
