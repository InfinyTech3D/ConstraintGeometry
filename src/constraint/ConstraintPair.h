#pragma once

#include <Constraint.h>

namespace sofa {

namespace core {

namespace behavior {

class ConstraintPair : public Constraint {
public :

    Data<std::string> d_from;
    Data<std::string> d_dest;

    SOFA_CLASS(ConstraintPair, Constraint );

    ConstraintPair();

    void init();

    void processGeometricalData();

protected:
    BaseGeometry * m_from;
    BaseGeometry * m_dest;
};

} // namespace controller

} // namespace component

} // namespace sofa
