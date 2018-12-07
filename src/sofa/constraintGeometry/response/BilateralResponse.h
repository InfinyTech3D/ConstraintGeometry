#pragma once

#include <sofa/constraintGeometry/BaseResponse.h>
#include <math.h>

namespace sofa {

namespace constraintGeometry {

template<int C>
class BilateralConstraintResolution : public sofa::core::behavior::ConstraintResolution {
public:
    BilateralConstraintResolution(double m) : ConstraintResolution(C), m_maxForce(m) {}

    virtual void init(int line, double** w, double */*force*/)
    {
        sofa::defaulttype::Mat<C,C,double> temp;
        for (unsigned j=0;j<C;j++) {
            for (unsigned i=0;i<C;i++) {
                temp[j][i] = w[line+j][line+i];
            }
        }
        invertMatrix(invW, temp);
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/)
    {
        for(int i=0; i<C; i++) {
            for(int j=0; j<C; j++)
                force[line+i] -= d[line+j] * invW[i][j];
        }
    }

    static std::string Name() {
        std::string res;
        for (unsigned i=0;i<C;i++) res.append("B");
        return res;
    }

    double m_maxForce;
    sofa::defaulttype::Mat<C,C,double> invW;
};

template<int C>
class BilateralResponse : public TResponse<BilateralConstraintResolution<C> > {
public:
    SOFA_CLASS(SOFA_TEMPLATE(BilateralResponse,C) , SOFA_TEMPLATE(TResponse,BilateralConstraintResolution<C>));

    Data<double> d_maxForce;

    BilateralResponse()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force")) {}

    virtual core::behavior::ConstraintResolution* getConstraintResolution() {
        return new BilateralConstraintResolution<C>(d_maxForce.getValue());
    }

    virtual std::string getClassName() const {
        return std::string("ConstraintResponse") + BilateralConstraintResolution<C>::Name();
    }

    virtual unsigned size() {
        return C;
    }

    virtual ConstraintNormal createConstraintNormal(defaulttype::Vector3 mainDir) {
        if (C>=3) { // specific case where the problem is full constrained
            return ConstraintNormal::createFrame(defaulttype::Vector3(1,0,0));
        } else {
            ConstraintNormal cn = ConstraintNormal::createFrame(mainDir);
            cn.m_normals.resize(C);
            return cn;
        }
    }

};

}

}
