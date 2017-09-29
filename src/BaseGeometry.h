#ifndef SOFA_COMPONENT_CONSTRAINT_GEOMETRY_H
#define SOFA_COMPONENT_CONSTRAINT_GEOMETRY_H

#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "ConstraintProximity.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

namespace sofa {

namespace core {

namespace behavior {

class ElementIterator {
public:
    virtual void next() = 0;

    virtual bool end() = 0;

    virtual unsigned getId() = 0;
};

typedef std::shared_ptr<ElementIterator> ElementIteratorPtr;

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

    Data<defaulttype::Vec4f> d_color;

    class DefaultElementIterator : public ElementIterator {
    public:
        DefaultElementIterator(int nbe) {
            m_nbe = nbe;
            id = 0;
        }

        void next() {
            id++;
        }

        bool end() {
            return id>=m_nbe;
        }

        unsigned getId() {
            return id;
        }

    private:
        int id;
        int m_nbe;
    };

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vec4f(1,0.5,0,1), "color", "Color of the collision model")) {}

    core::topology::BaseMeshTopology* getTopology() const {
        return this->getContext()->getMeshTopology();
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getMstate() const {
        return dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes> *>(this->getContext()->getMechanicalState());
    }

    void init() {
        if (getTopology() == NULL) serr << "Error cannot find the topology" << sendl;
        if (getMstate() == NULL) serr << "Error cannot find the topology" << sendl;
    }

    virtual int getNbElements() const = 0;

    virtual ConstraintProximityPtr getElementProximity(unsigned eid) const = 0;

    void computeBBox(const core::ExecParams* params, bool /*onlyVisible*/)  {
        SReal minBBox[3] = {1e10,1e10,1e10};
        SReal maxBBox[3] = {-1e10,-1e10,-1e10};

        helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
        for (unsigned i=0;i<x.size();i++) {
            for (int c=0; c<3; c++)
            {
                if (x[i][c] > maxBBox[c]) maxBBox[c] = x[i][c];
                if (x[i][c] < minBBox[c]) minBBox[c] = x[i][c];
            }
        }

        this->f_bbox.setValue(params,sofa::defaulttype::TBoundingBox<SReal>(minBBox,maxBBox));
    }

    ElementIteratorPtr getElementIterator() {
        return ElementIteratorPtr(new DefaultElementIterator(getNbElements()));
    }

    Eigen::MatrixXd pinv(const Eigen::MatrixXd & m) const {
        double epsilon= 1e-15;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singVals = svd.singularValues();
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType invSingVals = singVals;
        for(int i=0; i<singVals.rows(); i++) {
            if(singVals(i)*singVals(i) <= epsilon*epsilon) invSingVals(i) = 0.0;
            else invSingVals(i) = 1.0 / invSingVals(i);
        }
        Eigen::MatrixXd S_inv = invSingVals.asDiagonal();
        Eigen::MatrixXd m_inverse = svd.matrixV()*S_inv* svd.matrixU().transpose();
        return m_inverse;
    }

    void projectPoint(const Coord & Q,ConstraintProximity * pinfo) const {
        const int maxIt = 30;
        const double tolerance = 0.0001;
        const double threshold = 0.0000001;
        double delta = 0.001;

        helper::vector<defaulttype::Vector3> controlPoints;
        pinfo->getControlPoints(controlPoints);

        if (controlPoints.size() <= 1) return;

        //check the control points that are necessary to activate
        helper::vector<bool> usePoints;
        usePoints.resize(controlPoints.size());

        int it = 0;

        while (it< maxIt) {
            defaulttype::Vector3 P = pinfo->getPosition();
            helper::vector<defaulttype::Vector3> normals;

            for (unsigned i=0;i<pinfo->m_fact.size();i++) {
                if (pinfo->m_fact[i] == 0) usePoints[i] = dot(Q - P,controlPoints[i] - P) > 0;
                else usePoints[i] = true;

                if (usePoints[i]) normals.push_back((controlPoints[i] - P)*delta);
            }

            unsigned JLin = normals.size();

            if (JLin == 0) break;

            Eigen::VectorXd e0(JLin);
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(JLin,JLin);

            defaulttype::Vector3 PQ = Q-P;

            double err=0.0;
            for (unsigned j=0;j<JLin;j++) {
                double e = dot(PQ,normals[j]);

                e0(j) = e;

                err += e*e;
            }

            if (sqrt(err)<tolerance) break;

            for (unsigned j=0;j<JLin;j++) {
                const defaulttype::Vector3 R = Q-(P+normals[j]);
                for (unsigned i=0;i<JLin;i++) {
                    const double fxdx = dot(R, normals[i]);
                    J(i,j) = (fxdx - e0(i))/ delta;
                }
            }

    //        std::cout << "e0=\n" << e0 << std::endl;
    //        std::cout << "J=\n" << J << std::endl;

            Eigen::MatrixXd invJ = pinv(J);
            Eigen::VectorXd dx = -invJ * e0;

            helper::vector<double> Dx;

            int k=0;
            for (unsigned i=0;i<usePoints.size();i++) {
                if (usePoints[i]) Dx.push_back(dx(k++));
                else Dx.push_back(0.0);
            }

            helper::vector<double> prev = pinfo->m_fact;

            it++;
            pinfo->inc(Dx);

            double res = 0.0;
            for (unsigned i=0;i<pinfo->m_fact.size();i++) res += std::pow((prev[i] - pinfo->m_fact[i]),2);
            if (sqrt(res) < threshold) break;

    //                std::cout << "DX(" << sqrt(res) << ")=" << Dx << "    |||||| " << normals << std::endl;
        }


    //            double sum = 0.0;
    //            for (unsigned i=0;i<m_fact.size();i++) sum += m_fact[i];
    //            std::cout << "NEWTON ITERATIONS " << it << " sum=" << sum << " fact=" << m_fact << std::endl;
    }

private :
    void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }

protected:

    virtual void prepareDetection() {}

};





} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
