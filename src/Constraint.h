//#pragma once

//#include <BaseGeometry.h>
//#include <Response.h>

//namespace constraintGeometry {

//class Constraint : public BaseConstraint {
//public:
//    Data<std::string> d_response;
//    Data<std::string> d_algo;

//    Constraint()
//    : d_response(initData(&d_response, "response", "Response"))
//    , d_algo(initData(&d_algo, "algo", "Algorithm")) {}

//    void init() {
//        this->getContext()->get(m_response,d_response.getValue());
//        if (m_response == NULL) serr << "Error cannot find the reponse" << std::endl;

//        this->getContext()->get(m_algo,d_algo.getValue());
//        if (m_algo == NULL) serr << "Error cannot find the reponse" << std::endl;
//    }

//    void buildConstraintMatrix(const ConstraintParams* cParams, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
//        m_constraints.clear();
//        if (m_response == NULL) return;

//        m_response->getConstraintNormals(m_detections,m_constraints);

//        for (unsigned i=0;i<m_constraints.size();i++) {
//            m_constraints[i].buildConstraintMatrix(cParams,cId,constraintId);
//            constraintId += m_constraints[i].size();
//        }
//    }

//    void getConstraintViolation(const ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned cid) {
//        for (unsigned i=0;i<m_constraints.size();i++) {
//            m_response->getConstraintViolation(m_detections[i], m_constraints[i], v, cid);
//            cid+=m_constraints[i].size();
//        }
//    }

//    void getConstraintResolution(const ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
//        for (unsigned i=0;i<m_constraints.size();i++) {
//            resTab[offset] = m_response->getResolution(m_constraints[i]);
//            offset+=m_constraints[i].size();
//        }
//    }

//    void draw(const core::visual::VisualParams* vparams) {
//        if (! vparams->displayFlags().getShowInteractionForceFields()) return;

//        for (unsigned i=0;i<m_detections.size();i++) {

//            glBegin(GL_LINES);

//            defaulttype::Vector3 P = m_detections[i].first->getPosition();
//            defaulttype::Vector3 Q = m_detections[i].second->getPosition();

//            helper::gl::glVertexT(P);
//            helper::gl::glVertexT(Q);

//            glEnd();
//        }

//        for (unsigned i=0;i<m_constraints.size();i++) {
//            defaulttype::Vector3 Q = m_detections[i].second->getPosition();

//            for (unsigned n=0;n<m_constraints[i].normals().size();n++) {
//                vparams->drawTool()->drawArrow(Q,Q + m_constraints[i].normals()[n] * 1, 0.1, defaulttype::Vec4f(1.0f,0.0f,0.0f,1.0f));
//            }
//        }

//    }

//    void updateForceMask() {}

//protected:
//    CollisionAlgorithm * m_algo;
//    Response * m_response;
//    PariProximityVector m_detections;
//    helper::vector<ConstraintNormal> m_constraints;
//};

//}
