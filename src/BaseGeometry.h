#ifndef SOFA_COMPONENT_CONSTRAINT_GEOMETRY_H
#define SOFA_COMPONENT_CONSTRAINT_GEOMETRY_H

#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "ConstraintProximity.h"


namespace sofa {

namespace core {

namespace behavior {

class ElementIterator {
public:
    virtual bool next() = 0;

    virtual unsigned getId() = 0;

    virtual const BaseGeometry * getGeometry() = 0;

    virtual ConstraintProximityPtr getElementProximity() = 0;

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
        DefaultElementIterator(BaseGeometry * geo) {
            m_id = 0;
            m_geo = geo;
        }

        bool next() {
            m_id++;
            return m_id<m_geo->getNbElements();
        }

        unsigned getId() {
            return m_id;
        }

        BaseGeometry * getGeometry() {
            return m_geo;
        }

        ConstraintProximityPtr getElementProximity() {
            return m_geo->getElementProximity(m_id);
        }

    private:
        int m_id;
        BaseGeometry * m_geo;
    };

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vec4f(1,0.5,0,1), "color", "Color of the collision model")) {}

    core::topology::BaseMeshTopology* getTopology() const {
        return this->getContext()->getMeshTopology();
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getMstate() const {
        return dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes> *>(this->getContext()->getState());
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
        return std::make_shared<DefaultElementIterator>(this);
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
