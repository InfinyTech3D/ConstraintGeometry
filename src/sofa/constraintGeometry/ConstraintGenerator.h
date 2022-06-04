#pragma once

#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa::constraintGeometry {

class ConstraintGenerator : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(ConstraintGenerator, sofa::core::objectmodel::BaseObject);

    Data<collisionAlgorithm::DetectionOutput<collisionAlgorithm::BaseProximity,collisionAlgorithm::BaseProximity>> d_input;
    Data<collisionAlgorithm::DetectionOutput<ConstraintProximity,ConstraintProximity> > d_output;

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

            auto & pairs = *d_output.beginEdit();
            pairs.clear();
            for (unsigned i=0;i<d_input.getValue().size();i++) {
                auto & pair = d_input.getValue()[i];

                ConstraintProximity::SPtr first_cstProx = l_firstHandler->createConstraintProximity(pair.first);
                if (first_cstProx==NULL) continue;

                ConstraintProximity::SPtr second_cstProx = l_secondHandler->createConstraintProximity(pair.second);
                if (second_cstProx==NULL) continue;

                pairs.add(first_cstProx,second_cstProx);
            }
            d_output.endEdit();
        });
    }

};

}
