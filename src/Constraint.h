#ifndef SOFA_COMPONENT_CONSTRAINT_ConstraintResponse_H
#define SOFA_COMPONENT_CONSTRAINT_ConstraintResponse_H

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
#include "ConstraintNormal.h"
#include "CollisionAlgorithm.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/gl/template.h>


namespace sofa {

namespace core {

namespace behavior {

class ConstraintResponse : public core::BehaviorModel {
public :

    virtual void processAlgorithm(helper::vector<ConstraintNormal> & cn) = 0;

};

class Constraint : public sofa::core::behavior::BaseConstraint {
public :

    SOFA_CLASS(Constraint, sofa::core::behavior::BaseConstraint );

public:

    Data<std::string> d_algorithm;

    Constraint()
    : d_algorithm(initData(&d_algorithm, "algorithm", "Collision reponse algorithm")) {}

    void init () {
        this->getContext()->get(m_algorithm,d_algorithm.getValue());
        if (m_algorithm == NULL) serr << "Error cannot find the reponse" << std::endl;
    }

    void processGeometricalData() {
        m_constraints.clear();

        if (m_algorithm == NULL) return;

        m_algorithm->processAlgorithm(m_constraints);
    }

    void buildConstraintMatrix(const ConstraintParams* cParams, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->buildConstraintMatrix(cParams,cId,constraintId);
            constraintId+=m_constraints[i]->size();
        }
    }

    void getConstraintViolation(const ConstraintParams* cParams, defaulttype::BaseVector *v,unsigned cid) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->getConstraintViolation(cParams,v,cid);
            cid+=m_constraints[i]->size();
        }
    }

    void getConstraintResolution(const ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            resTab[offset] = m_constraints[i]->getResolution();
            offset+=m_constraints[i]->size();
        }
    }

    void draw(const core::visual::VisualParams* vparams) {
        for (unsigned i=0;i<m_constraints.size();i++) {

            glBegin(GL_LINES);

            defaulttype::Vector3 P = m_constraints[i]->getP1()->getPosition();
            defaulttype::Vector3 Q = m_constraints[i]->getP2()->getPosition();

            helper::gl::glVertexT(P);
            helper::gl::glVertexT(Q);

            glEnd();

            for (unsigned n=0;n<m_constraints[i]->getNormals().size();n++) {
                defaulttype::Vector3 N = m_constraints[i]->getNormals()[n];

                vparams->drawTool()->drawArrow(Q,Q + N,1.0/20.0, defaulttype::Vec<4,float>(0.0f,0.0f,1.0f,1.0f));
            }
        }
    }

    void updateForceMask() {}

private:
    helper::vector<ConstraintNormalPtr> m_constraints;
    CollisionAlgorithm * m_algorithm;

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
