#ifndef SOFA_COMPONENT_CONSTRAINT_GEOMETRY_H
#define SOFA_COMPONENT_CONSTRAINT_GEOMETRY_H

#include <math.h>
#include <sofa/defaulttype/Vec.h>
//#include "ConstraintProximity.h"
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/collision/Pipeline.h>
#include <SofaBaseTopology/PointSetTopologyContainer.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa {

namespace core {

namespace behavior {

class BaseGeometry : public sofa::core::objectmodel::BaseObject {
    friend class ConstraintProximity;

public :
    SOFA_CLASS(BaseGeometry, sofa::core::objectmodel::BaseObject);

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

    class ConstraintProximity {
    public :

        typedef BaseGeometry::DataMatrixDeriv  DataMatrixDeriv;
        typedef BaseGeometry::MatrixDeriv  MatrixDeriv;
        typedef BaseGeometry::MatrixDerivRowIterator  MatrixDerivRowIterator;

        virtual defaulttype::Vector3 getPosition(core::VecCoordId vid = core::VecCoordId::position()) const = 0;

        virtual void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) = 0;

////        bool operator ==(const ConstraintProximity & b) const {
////            if (m_fact.size() != b.m_fact.size()) return false;

////            for (unsigned i=0;i<m_fact.size();i++) {
////                bool find = false;
////                unsigned j=0;
////                while (j<m_fact.size() && !find) {
////                    find = m_pid[i] == b.m_pid[j] && m_fact[i] == b.m_fact[j];
////                    j++;
////                }
////                if (!find) return false;
////            }

////            return true;
////        }

////        inline friend std::ostream& operator << ( std::ostream& out, const ConstraintProximity& c ) {
////            for (unsigned i=0;i<c.m_fact.size();i++) {
////                out << "[" << c.m_pid[i] << "," << c.m_fact[i] << "]";
////            }
////            return out;
////        }

    };

    typedef std::shared_ptr<ConstraintProximity> ConstraintProximityPtr;

    class ConstraintElement {
    public:
        // this function returns a proximity at the center of the element
        virtual ConstraintProximityPtr getDefaultProximity() = 0;

        //this function returns a vector with all the control points of the element
        virtual helper::vector<ConstraintProximityPtr> getConstrolPoints() = 0;

        //this function project the point P on the element and return the corresponding proximity
        virtual ConstraintProximityPtr project(defaulttype::Vector3 P) = 0;
    };

    typedef std::shared_ptr<ConstraintElement> ConstraintElementPtr;

    class ElementIterator {
    public:
        virtual bool next() = 0;

        virtual ConstraintElementPtr getElement() = 0;
    };

    typedef std::shared_ptr<ElementIterator> ElementIteratorPtr;

    class DefaultElementIterator : public ElementIterator {
    public:
        DefaultElementIterator(const BaseGeometry * geo) {
            m_id = 0;
            m_geo = geo;
        }

        bool next() {
            m_id++;
            return m_id<m_geo->getNbElements();
        }

        ConstraintElementPtr getElement() {
            return m_geo->getElement(m_id);
        }

    private:
        unsigned m_id;
        const BaseGeometry * m_geo;
    };

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vec4f(1,0.5,0,1), "color", "Color of the collision model")) {
//        m_dirty=true;
        f_listening.setValue(true);
    }

    core::topology::BaseMeshTopology* getTopology() const {
        return this->getContext()->getMeshTopology();
    }

    sofa::core::behavior::MechanicalState<BaseGeometry::DataTypes> * getMstate() const {
        return dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes> *>(this->getContext()->getState());
    }

    defaulttype::Vector3 getPos(unsigned pid,core::VecCoordId vid = core::VecCoordId::position()) const {
        helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(vid);
        return x[pid];
    }


    void computeBBox(const core::ExecParams* params, bool /*onlyVisible*/)  {
        m_dirty=true;

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

    ElementIteratorPtr getElementIterator() const {
        return std::make_shared<DefaultElementIterator>(this);
    }

    virtual unsigned getNbElements() const = 0;

    virtual ConstraintElementPtr getElement(unsigned i) const = 0;


    void handleEvent(sofa::core::objectmodel::Event* event) {
        if (dynamic_cast<simulation::AnimateBeginEvent*>(event)) update();
        else if (dynamic_cast<simulation::AnimateEndEvent*>(event)) m_dirty = true;
    }

    void update() {
        if (m_dirty) {
            prepareDetection();
            m_dirty = false;
        }
    }


protected:
    virtual void prepareDetection() {}

    bool m_dirty;

};

typedef std::shared_ptr<BaseGeometry::ConstraintProximity> ConstraintProximityPtr;
typedef std::shared_ptr<BaseGeometry::ConstraintElement> ConstraintElementPtr;
typedef std::shared_ptr<BaseGeometry::ElementIterator> ElementIteratorPtr;

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
