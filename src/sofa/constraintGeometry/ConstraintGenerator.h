#pragma once

#include <sofa/constraintGeometry/ConstraintProximity.h>
#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/constraintGeometry/ConstraintProximityOperation.h>

namespace sofa::constraintGeometry {

class ConstraintPairsOutput {
public:

    typedef std::pair<ConstraintProximity::SPtr,ConstraintProximity::SPtr> ConstraintPairs;

    friend std::ostream& operator<<(std::ostream& os, const ConstraintPairsOutput& )  { return os; }

    friend std::istream& operator>>(std::istream& i, ConstraintPairsOutput& ) { return i; }

    inline unsigned size() const { return m_constraints.size(); }

    inline void clear() { m_constraints.clear(); }

    inline const ConstraintPairs & operator[] (unsigned i) const { return m_constraints[i]; }

    inline void push_back(const std::pair<ConstraintProximity::SPtr,ConstraintProximity::SPtr> & p) { m_constraints.push_back(p); }

private:
    std::vector<ConstraintPairs> m_constraints;
};

class ConstraintGenerator : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(ConstraintGenerator, sofa::core::objectmodel::BaseObject);

    Data<collisionAlgorithm::DetectionOutput> d_input;
    Data<ConstraintPairsOutput> d_output;

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

            ConstraintPairsOutput & pairs = *d_output.beginEdit();
            pairs.clear();
            for (unsigned i=0;i<d_input.getValue().size();i++) {
                auto & pair = d_input.getValue()[i];

                ConstraintProximityOperation::FUNC firstOp = ConstraintProximityOperation::get(l_firstHandler->getTypeInfo(),pair.first->getTypeInfo());
                ConstraintProximity::SPtr first_cstProx = firstOp(l_firstHandler.get(), pair.first);
                if (first_cstProx==NULL) continue;

                ConstraintProximityOperation::FUNC secondOp = ConstraintProximityOperation::get(l_secondHandler->getTypeInfo(),pair.second->getTypeInfo());
                ConstraintProximity::SPtr second_cstProx = secondOp(l_secondHandler.get(), pair.second);
                if (second_cstProx==NULL) continue;

                pairs.push_back(std::pair<ConstraintProximity::SPtr,ConstraintProximity::SPtr>(first_cstProx,second_cstProx));
            }
            d_output.endEdit();
        });
    }

};

}
