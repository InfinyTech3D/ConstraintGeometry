#ifndef SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHMS_H
#define SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHMS_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/collision/Pipeline.h>
#include <SofaBaseTopology/PointSetTopologyContainer.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include "BaseGeometry.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/gl/template.h>

namespace sofa {

namespace core {

namespace behavior {

typedef std::pair<ConstraintProximityPtr,ConstraintProximityPtr> PariProximity;
typedef helper::vector<PariProximity > PariProximityVector;

//class CollisionResponse : public core::BehaviorModel {
//public:

//    virtual void fillConstraints(helper::vector<ConstraintNormalPtr> & cn) = 0;

//    void updatePosition(SReal /*dt*/) {
//        prepareDetection();
//    }

//protected:

//    virtual void prepareDetection() {}

//};

class CollisionAlgorithm : public sofa::core::objectmodel::BaseObject {
public :
    SOFA_CLASS(CollisionAlgorithm, sofa::core::objectmodel::BaseObject);

    Data<std::string> d_from;
    Data<std::string> d_dest;

    CollisionAlgorithm()
    : d_from(initData(&d_from, "from", "Collision reponse algorithm"))
    , d_dest(initData(&d_dest, "dest", "Collision reponse algorithm")) {}

    virtual void processAlgorithm() = 0;

    void init() {
        this->getContext()->get(m_from,d_from.getValue());
        if (m_from == NULL) serr << "Error cannot find the reponse" << std::endl;

        this->getContext()->get(m_dest,d_dest.getValue());
        if (m_dest == NULL) serr << "Error cannot find the reponse" << std::endl;

        m_dirty = true;
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowInteractionForceFields()) return;

        for (unsigned i=0;i<m_pairDetection.size();i++) {

            glBegin(GL_LINES);

            defaulttype::Vector3 P = m_pairDetection[i].first->getPosition();
            defaulttype::Vector3 Q = m_pairDetection[i].second->getPosition();

            helper::gl::glVertexT(P);
            helper::gl::glVertexT(Q);

            glEnd();
        }
    }

    virtual void clear() {
        m_pairDetection.clear();
        m_dirty = true;
    }

    virtual void update() {
        if (! m_dirty) return;

        clear();
        m_dirty = false;

        if (m_from == NULL) return;
        if (m_dest == NULL) return;


        m_from->update();
        m_dest->update();

        processAlgorithm();
    }

    void handleEvent(sofa::core::objectmodel::Event* event) {
        if (dynamic_cast<simulation::AnimateBeginEvent*>(event)) update();
        else if (dynamic_cast<simulation::AnimateEndEvent*>(event)) m_dirty = true;
    }

    const PariProximityVector & getDetection() {
        return m_pairDetection;
    }


protected:
    PariProximityVector m_pairDetection;
    BaseGeometry * m_from;
    BaseGeometry * m_dest;
    bool m_dirty;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
