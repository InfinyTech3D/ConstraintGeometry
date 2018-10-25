#pragma once

#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

class UnilateralConstraintResolution : public sofa::core::behavior::ConstraintResolution {
public:
    UnilateralConstraintResolution(double m) : ConstraintResolution(1), m_maxForce(m){}

    virtual void resolution(int line, double** w, double* d, double* force, double * /*dFree*/) {
        force[line] -= d[line] / w[line][line];

        if (force[line]>m_maxForce) force[line] = m_maxForce;
        else if (force[line]<0) force[line] = 0.0;
    }

    static std::string Name() {
        return "U";
    }

    double m_maxForce;
};

class UnilateralFrictionResolution : public sofa::core::behavior::ConstraintResolution {
public:
    UnilateralFrictionResolution(double m,double f) : ConstraintResolution(3), m_maxForce(m), m_friction(f) {}

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

    static std::string Name() {
        return "UFF";
    }

    double m_maxForce;
    double m_friction;
    sofa::defaulttype::Mat<3,3,double> invW;
};

class ContactResponseUFF : public TResponse<UnilateralFrictionResolution> {
public:
    SOFA_CLASS(ContactResponseUFF , SOFA_TEMPLATE(TResponse,UnilateralFrictionResolution));

    Data<double> d_maxForce;
    Data<double> d_friction;

    ContactResponseUFF()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "friction", "Friction")) {}

    virtual core::behavior::ConstraintResolution* getConstraintResolution() {
        if (d_friction.getValue() == 0.0) return new UnilateralConstraintResolution(d_maxForce.getValue());
        return new UnilateralFrictionResolution(d_maxForce.getValue(), d_friction.getValue());
    }

    virtual unsigned size() {
        if (d_friction.getValue() == 0.0) return 1;
        return 3;
    }

};

class ContactResponseU : public TResponse<UnilateralConstraintResolution> {
public:
    SOFA_CLASS(ContactResponseU , SOFA_TEMPLATE(TResponse,UnilateralConstraintResolution));

    Data<double> d_maxForce;

    ContactResponseU()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    virtual core::behavior::ConstraintResolution* getConstraintResolution() {
        return new UnilateralConstraintResolution(d_maxForce.getValue());
    }


    unsigned size() {
        return 1;
    }
};

}

}
