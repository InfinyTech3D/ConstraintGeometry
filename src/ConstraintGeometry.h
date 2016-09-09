#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_H
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace core {

namespace behavior {

class ConstraintProximity {
public :

    void clear() {
        pid.clear();
        fact.clear();
    }

    unsigned size() const {
        return pid.size();
    }

    void add(unsigned id,double f) {
        pid.push_back(id);
        fact.push_back(f);
    }

    helper::vector<unsigned> pid;
    helper::vector<double> fact;
};

class ConstraintNormal {
public:

    void clear() {
        normals.clear();
    }

    void add(const defaulttype::Vector3 & P) {
        normals.push_back(P);
    }

    unsigned size() const {
        return normals.size();
    }

    helper::vector<defaulttype::Vector3> normals;

};

template<class DataTypes>
class ConstraintGeometry : public sofa::core::objectmodel::BaseObject {
public :
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    SOFA_CLASS(ConstraintGeometry, sofa::core::objectmodel::BaseObject);

    virtual sofa::core::behavior::MechanicalState<DataTypes> * getMstate() = 0 ;

    virtual Coord getContactPosition(const ConstraintProximity & sinfo) {
        Coord res;

        const helper::ReadAccessor<Data <VecCoord> >& x = *getMstate()->read(core::VecCoordId::position());

        for (unsigned p=0;p<sinfo.pid.size();p++) {
            if (sinfo.fact[p]!=0.0) res+=x[sinfo.pid[p]] * sinfo.fact[p];
        }

        return res;
    }

    virtual Coord getContactFreePosition(const ConstraintProximity & sinfo) {
        Coord res;

        const helper::ReadAccessor<Data <VecCoord> >& xfree = *getMstate()->read(core::VecCoordId::freePosition());

        for (unsigned p=0;p<sinfo.pid.size();p++) {
            if (sinfo.fact[p]!=0.0) res+=xfree[sinfo.pid[p]] * sinfo.fact[p];
        }

        return res;
    }

    void addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const ConstraintNormal & ninfo,bool direct) {
        DataMatrixDeriv & c_d = *cId[getMstate()].write();

        MatrixDeriv & c = *c_d.beginEdit();

        double coeff = (direct) ? 1.0 : -1.0;

        for (unsigned i=0;i<ninfo.normals.size();i++) {
            MatrixDerivRowIterator c_it1 = c.writeLine(cline+i);

            for (unsigned p=0;p<pinfo.pid.size();p++) {
                if (pinfo.fact[p] == 0.0) continue;
                c_it1.addCol(pinfo.pid[p], ninfo.normals[i] * pinfo.fact[p] * coeff);
            }
        }

        c_d.endEdit();
    }

};

template<class DataType1,class DataType2>
static void addConstraint(core::MultiMatrixDerivId cId,unsigned & cline,ConstraintGeometry<DataType1> * cg1,const ConstraintProximity & pinfo1,ConstraintGeometry<DataType2> * cg2,const ConstraintProximity & pinfo2,const ConstraintNormal & ninfo) {
    cg1->addConstraint(cId,cline,pinfo1,ninfo,false);
    cg2->addConstraint(cId,cline,pinfo2,ninfo,true);
    cline += ninfo.size();
}

template<class DataType1>
static void addConstraint(core::MultiMatrixDerivId cId,unsigned & cline,ConstraintGeometry<DataType1> * cg,const ConstraintProximity & pinfo,const ConstraintNormal & ninfo) {
    cg->addConstraint(cId,cline,pinfo,ninfo,false);
    cline += ninfo.size();
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
