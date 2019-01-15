#pragma once

#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/defaulttype/Mat.h>

namespace sofa {

namespace constraintGeometry {

class PenetrationConstraintResolution : public core::behavior::ConstraintResolution
{
public:
    PenetrationConstraintResolution(double mus, double maf,double muf,double mub, bool & p)
    : ConstraintResolution(3)
    , penetrated(p) {
        _mu_tan = mus;
        _max_force = maf;
        _mu_front = muf;
        _mu_back = mub;
    }

    virtual void init(int line, double** w, double */*force*/)
    {
        sofa::defaulttype::Mat<3,3,double> temp;
        temp[0][0] = w[line+0][line+0];
        temp[0][1] = w[line+0][line+1];
        temp[0][2] = w[line+0][line+2];
        temp[1][0] = w[line+1][line+0];
        temp[1][1] = w[line+1][line+1];
        temp[1][2] = w[line+1][line+2];
        temp[2][0] = w[line+2][line+0];
        temp[2][1] = w[line+2][line+1];
        temp[2][2] = w[line+2][line+2];

        invertMatrix(invW, temp);
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/)
    {
        //compute the force as bilateral
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                force[line+i] -= d[line+j] * invW[i][j];

        if(force[line]<0) {
            force[line+0]=0;
            force[line+1]=0;
            force[line+2]=0;
            penetrated=false;
        } else if (force[line]<_max_force) {

            double friction = _mu_tan * force[line+0];

            if (force[line+1]>friction) force[line+1] = friction;
            else if (force[line+1]<-friction) force[line+1] = -friction;

            if (force[line+2]>friction) force[line+2] = friction;
            else if (force[line+2]<-friction) force[line+2] = -friction;
            penetrated=false;
        } else {
            //if (force[line]<0) force[line] *= _mu_back;
            //else force[line] *= _mu_front;
            force[line+0] = _max_force + (force[line+0] - _max_force) * _mu_front;
            //            force[line+1]=0;
            //            force[line+2]=0;
            penetrated=true;
        }
    }

    virtual void store(int /*line*/, double* /*force*/, bool /*convergence*/) {
        //    penetrated=force[line]>=_max_force;
    }

protected:

    double _mu_tan;
    double _max_force;
    double _mu_front;
    double _mu_back;
    sofa::defaulttype::Mat<3,3,double> invW;
    bool & penetrated;
};

class TrajectoryConstraintResolution : public core::behavior::ConstraintResolution
{
public:
    TrajectoryConstraintResolution(double mf,double mb, double mfact) : ConstraintResolution(3) {
        _mu_front = mf;
        _mu_back = mb;
        _factor = mfact;
    }

    virtual void init(int line, double** w, double */*force*/)
    {
        sofa::defaulttype::Mat<3,3,double> temp;
        temp[0][0] = w[line+0][line+0];
        temp[0][1] = w[line+0][line+1];
        temp[0][2] = w[line+0][line+2];
        temp[1][0] = w[line+1][line+0];
        temp[1][1] = w[line+1][line+1];
        temp[1][2] = w[line+1][line+2];
        temp[2][0] = w[line+2][line+0];
        temp[2][1] = w[line+2][line+1];
        temp[2][2] = w[line+2][line+2];

        invertMatrix(invW, temp);
    }

    virtual void resolution(int line, double** /*w*/, double* d, double* force, double * /*dFree*/)
    {
        //compute the force as bilateral
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                force[line+i] -= d[line+j] * invW[i][j];

        if (force[line]<0) force[line] *= _mu_back;
        else force[line] *= _mu_front;

        force[line+0]*=_factor;
        force[line+1]*=_factor;
        force[line+2]*=_factor;
    }

protected:
    sofa::defaulttype::Mat<3,3,double> invW;
    double _mu_front;
    double _mu_back;
    double _factor;

};

}

}
