#pragma once

#include <CollisionAlgorithm/CollisionPipeline.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <ConstraintGeometry/ConstraintProximity.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <ConstraintGeometry/operations/ConstraintProximityOperation.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>

namespace sofa ::constraintGeometry {

/*!
 * \brief The BaseConstraint abstract class is the implementation of sofa's abstract BaseConstraint
 */
class BaseNormalHandler : public collisionAlgorithm::CollisionComponent {
public:
    SOFA_ABSTRACT_CLASS(BaseNormalHandler, collisionAlgorithm::CollisionComponent);

    typedef collisionAlgorithm::BaseGeometry BaseGeometry;
    typedef collisionAlgorithm::BaseProximity BaseProximity;

    void init() {
        if (getGeometry()==NULL) {
            std::cerr << "Error cannot find the geometry" << std::endl;
            return;
        }

        getGeometry()->addSlave(this);
    }

    virtual BaseGeometry * getGeometry() = 0;

    virtual const std::type_info & getTypeInfo() = 0;

    virtual ConstraintProximity::SPtr createConstraintProximity(const BaseProximity::SPtr & prox) {
        ConstraintProximityOperation::FUNC operation = ConstraintProximityOperation::get(getTypeInfo(),prox->getTypeInfo());
        return operation(this, prox);
    }

    void draw(const core::visual::VisualParams * vparams) override {
        if (! vparams->displayFlags().getShowNormals()) return;

        type::RGBAColor color(1,0,0,1);
        double scale = 1.0;

        glDisable(GL_LIGHTING);
        if (color[3] == 0.0) return;

        collisionAlgorithm::Operations::CreateCenterProximity::FUNC operation = collisionAlgorithm::Operations::CreateCenterProximity::Operation::get(getGeometry()->pointBegin());

        for (auto it = getGeometry()->pointBegin();it != getGeometry()->end(); it++) {
            auto prox = operation(it->element());
            auto cp = createConstraintProximity(prox);

            vparams->drawTool()->drawArrow(cp->getPosition(), cp->getPosition() - cp->getNormal() * scale, scale * 0.1, color);
        }
    }

};

}
