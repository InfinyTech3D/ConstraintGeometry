#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/directions/BindDirection.h>
#include <sofa/constraintGeometry/constraint/InsertionResolution.h>

namespace sofa::constraintGeometry {

class ConstraintInsertion : public TBaseConstraint<collisionAlgorithm::BaseProximity,collisionAlgorithm::BaseProximity> {
public:
    SOFA_CLASS(ConstraintInsertion , SOFA_TEMPLATE2(TBaseConstraint,BaseProximity,BaseProximity));

    Data<SReal> d_frictionCoeff;
    Data<sofa::type::vector<double> > d_maxForce;
    Data<sofa::type::vector<double> > d_compliance;
    core::objectmodel::SingleLink<ConstraintInsertion,ConstraintDirection, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_directions;

    ConstraintInsertion()
    : d_frictionCoeff(initData(&d_frictionCoeff, 0.0, "frictionCoeff" , "static friction coefficient (should be less than 1)"))
    , d_maxForce(initData(&d_maxForce, "maxForce", "Max force"))
    , d_compliance(initData(&d_compliance, "compliance", "Max force"))
    , l_directions(initLink("directions", "link to the default direction")) {}

    void init() { // make sure we have a direction
        if (l_directions == NULL) {
            msg_error(this) << "Error you must provide the directions to solve the constraints ";
        } else this->addSlave(l_directions.get());
    }

    virtual ConstraintNormal createConstraintNormal(const BaseProximity::SPtr & first, const BaseProximity::SPtr & second) const override {
        if (l_directions == NULL) return ConstraintNormal();
        return l_directions->createConstraintsNormal(first, second)
            .addOrthogonalDirection().addOrthogonalDirection();
    }

    core::behavior::ConstraintResolution* createConstraintResolution(const BaseInternalConstraint * cst) const {
        double maxf[3] = { std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        double comp[3] = { 0.0, 0.0, 0.0 };

        if (d_maxForce.getValue().size()>0) maxf[0]=d_maxForce.getValue()[0];
        if (d_maxForce.getValue().size()>1) maxf[1]=d_maxForce.getValue()[1];
        if (d_maxForce.getValue().size()>2) maxf[2]=d_maxForce.getValue()[2];

        if (d_compliance.getValue().size()>0) comp[0]=d_compliance.getValue()[0];
        if (d_compliance.getValue().size()>1) comp[1]=d_compliance.getValue()[1];
        if (d_compliance.getValue().size()>2) comp[2]=d_compliance.getValue()[2];

        return new InsertionConstraintResolution(d_frictionCoeff.getValue(), maxf[0],maxf[1],maxf[2],comp[0],comp[1],comp[2]);

        std::cerr << "Error the size of the constraint is not correct in ConstraintInsertion size=" << cst->size() << std::endl;
        return NULL;
    }

protected:

    sofa::type::vector<std::string> getBaseConstraintIdentifiers() override final
    {
        return { "Insertion" };
    }
};

}
