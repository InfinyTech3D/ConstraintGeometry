#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_INL
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTPROXIMITYT_INL

#include "BaseGeometry.h"
#include "ConstraintProximity.h"


namespace sofa {

namespace core {

namespace behavior {

void ConstraintProximity::getControlPoints(helper::vector<defaulttype::Vector3> & controlPoints) {
    helper::vector<double> prev = m_fact;
    m_fact.clear();
    m_fact.resize(prev.size(),0.0);
    for (unsigned i=0;i<m_fact.size();i++) {
        m_fact[i] = 1.0;
        controlPoints.push_back(getPosition());
        m_fact[i] = 0.0;
    }
    m_fact = prev;
}

void ConstraintProximity::inc(const helper::vector<double> & dir) {
    //apply dx
    for (unsigned i=0;i<dir.size();i++) m_fact[i]+=dir[i];

    //clamp the bary coord to the element
//            double sum = 0.0;
//            for (unsigned i=0;i<m_fact.size();i++) sum += m_fact[i];
//            if (sum == 0.0) return;
//            for (unsigned j=0;j<m_fact.size();j++) m_fact[j] *= 1.0/sum;

    for (unsigned i=0;i<m_fact.size();i++) {
        if (m_fact[i] < 0) {
            double remove = m_fact[i];
            m_fact[i] = 0;
            for (unsigned j=0;j<m_fact.size();j++) m_fact[j] /= (1.0 - remove);
//                    usePoints[i] = false;
//                    printf("CLAMP %d\n",i);
        }
    }

    double sum = 0.0;
    for (unsigned i=0;i<m_fact.size();i++) sum += m_fact[i];
    if (sum == 0.0) return;
    for (unsigned j=0;j<m_fact.size();j++) m_fact[j] *= 1.0/sum;

//            std::cout << "FACT (" << sum << ") = " << m_fact << std::endl;
}

void ConstraintProximity::refineToClosestPoint(const Coord & P) {
    m_geo->projectPoint(P,this);
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
