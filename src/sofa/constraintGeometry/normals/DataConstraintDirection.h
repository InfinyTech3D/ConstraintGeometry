#pragma once

#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/constraintGeometry/normals/ConstraintNormal.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint;

/*!
 * \brief The DataConstraintNormal class is the default class
 * generating a list of constraint directions
 * The class also serves as a template for directionGenerators
 * only the constructor and static defaultGetNormals should be redefined though
 */
class ConstraintDirection : public helper::vector<defaulttype::Vector3> {
public :
    typedef std::function<ConstraintNormal (const collisionAlgorithm::DetectionOutput::PairDetection & d)> GeneratorFunction;

    friend class sofa::core::objectmodel::DataValue<ConstraintDirection, true>;
    friend class sofa::core::objectmodel::DataValue<ConstraintDirection, false>;
    template<class T> friend class sofa::core::objectmodel::Data<T>::InitData;

    // std::placeholders::_1 is the first parameter
    ConstraintDirection(GeneratorFunction fct) : m_functor(fct) {}

    inline ConstraintNormal getConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d) const {
        if (size() == 0) return m_functor(d); // return the functor value
        return ConstraintNormal(*this); //use helper::vector Constructor
    }

protected:

    GeneratorFunction m_functor;

    // this is not allow to create this data without providing the functor
    ConstraintDirection() {}
};

}

}
