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
#include "ConstraintResponse.h"
#include "CollisionAlgorithm.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/gl/template.h>


namespace sofa {

namespace core {

namespace behavior {

template<class DataTypes>
class ConstraintResponse : public sofa::core::behavior::BaseConstraint {
public :

    SOFA_CLASS(ConstraintResponse, sofa::core::behavior::BaseConstraint );

public:

    Data <std::string> d_constraintAlgorithm;
    Data<std::string> d_from;
    Data<std::string> d_to;

    ConstraintResponse()
    : d_constraintAlgorithm(initData(&d_constraintAlgorithm, "algorithm", "Collision detection algorithm"))
    , d_from(initData(&d_from, "from", "From geometry"))
    , d_to(initData(&d_to, "to", "To geometry"))
    , m_response(this)
    {}

    void init() {
        this->getContext()->get(m_algo,d_constraintAlgorithm.getValue());
        if (m_algo == NULL) serr << "Cannot find the algorithm" << sendl;

        this->getContext()->get(m_geoFrom,d_from.getValue());
        if (m_geoFrom == NULL) serr << "Cannot find the from geometry\n" << sendl;

        this->getContext()->get(m_geoTo,d_to.getValue());
        if (m_geoTo == NULL) serr << "Cannot find the TO geometry\n" << sendl;

    }

    void processGeometricalData() {
        if (m_algo == NULL) return;
        if (m_geoFrom == NULL) return;
        if (m_geoTo == NULL) return;

        helper::vector<CollisionAlgorithm::PariProximity> pairs;
        m_algo->processAlgorithm(m_geoFrom,m_geoTo,pairs);

        m_constraints.clear();
        for (unsigned i=0;i<pairs.size();i++) {
            ConstraintNormal cn = DataTypes::makeConstraint(pairs[i]);
            if (!cn.empty()) m_constraints.push_back(cn);
        }
    }

    void buildConstraintMatrix(const ConstraintParams* cParams, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
//        for (unsigned i=0;i<m_constraints.size();i++) {
//            m_constraints[i].buildConstraintMatrix(cParams,cId,constraintId);
//        }
    }

    void getConstraintViolation(const ConstraintParams* cParams, defaulttype::BaseVector *v,unsigned cid) {
//        for (unsigned i=0;i<m_constraints.size();i++) {
//            m_constraints[i].getConstraintViolation(cParams,v,cid);
//        }
    }

    void getConstraintResolution(const ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
//        for (unsigned i=0;i<m_constraints.size();i++) {
//            resTab[offset] = m_response.getResolution();
//            offset+=m_constraints[i].size();
//        }
    }

    void draw(const core::visual::VisualParams* vparams) {
        for (unsigned i=0;i<m_constraints.size();i++) {

            if (m_constraints[i].getPinfo1() == NULL) return;
            if (m_constraints[i].getPinfo2() == NULL) return;

            glBegin(GL_LINES);


            defaulttype::Vector3 P = m_constraints[i].getPinfo1()->getPosition();
            defaulttype::Vector3 Q = m_constraints[i].getPinfo2()->getPosition();

            helper::gl::glVertexT(P);
            helper::gl::glVertexT(Q);

            glEnd();

            for (unsigned n=0;n<m_constraints[i].getNormals().size();n++) {
                defaulttype::Vector3 N = m_constraints[i].getNormals()[n];

                vparams->drawTool()->drawArrow(Q,Q + N,1.0/20.0, defaulttype::Vec<4,float>(0.0f,0.0f,1.0f,1.0f));
            }
        }
    }

    void updateForceMask() {}

public:
    CollisionAlgorithm * m_algo;
    BaseGeometry * m_geoFrom;
    BaseGeometry * m_geoTo;

    DataTypes m_response;
    helper::vector<ConstraintNormal> m_constraints;

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const ConstraintResponse<DataTypes>* = NULL)
    {
      return DataTypes::Name();
    }
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
