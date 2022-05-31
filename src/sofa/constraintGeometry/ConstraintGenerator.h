#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/constraintGeometry/InternalConstraint.h>
#include <sofa/constraintGeometry/ConstraintResponse.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>
#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/constraintGeometry/BaseNormalHandler.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintPairs {
public:

    friend std::ostream& operator<<(std::ostream& os, const ConstraintPairs& )  { return os; }

    friend std::istream& operator>>(std::istream& i, ConstraintPairs& ) { return i; }

    void clear() { m_constraints.clear(); }

    void push_back(const std::pair<ConstraintProximity::SPtr,ConstraintProximity::SPtr> & p) { m_constraints.push_back(p); }

private:
    std::vector<std::pair<ConstraintProximity::SPtr,ConstraintProximity::SPtr> > m_constraints;
};

class ConstraintGenerator : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(ConstraintGenerator, sofa::core::objectmodel::BaseObject);

    Data<collisionAlgorithm::DetectionOutput> d_input;
    Data<ConstraintPairs> d_output;

    core::objectmodel::DataCallback c_callback;

    core::objectmodel::SingleLink<ConstraintGenerator, BaseNormalHandler, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_firstHandler;
    core::objectmodel::SingleLink<ConstraintGenerator, BaseNormalHandler, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_secondHandler;

    ConstraintGenerator()
    : d_input(initData(&d_input, "input", "Link to detection output"))
    , d_output(initData(&d_output, "output", "Link to detection output"))
    , l_firstHandler(initLink("firstHandler", "link to the first normal handler"))
    , l_secondHandler(initLink("secondHandler", "link to the second normal handler"))
    {
        c_callback.addInput(&d_input);
        c_callback.addCallback([=](){

            ConstraintProximityOperation::FUNC firstOp = ConstraintProximityOperation::get(l_firstHandler->getTypeInfo());
            ConstraintProximityOperation::FUNC secondOp = ConstraintProximityOperation::get(l_secondHandler->getTypeInfo());

            ConstraintPairs & pairs = *d_output.beginEdit();
            pairs.clear();
            for (unsigned i=0;i<d_input.getValue().size();i++) {
                auto & pair = d_input.getValue()[i];

                pairs.push_back(std::pair<ConstraintProximity::SPtr,ConstraintProximity::SPtr>(firstOp(pair.first),
                                                                                               secondOp(pair.second)));
            }
            d_output.endEdit();
        });
    }

};

}

}
