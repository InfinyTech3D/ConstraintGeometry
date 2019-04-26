#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>
#include <sofa/constraintGeometry/normals/DataConstraintDirection.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The ConstraintUnilateral class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class ConstraintUnilateral : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintUnilateral , BaseConstraint);

    Data<double> d_friction;
    Data<collisionAlgorithm::DetectionOutput> d_input;
    Data<ConstraintDirection> d_direction;

    /*!
     * \brief ConstraintUnilateral constructor
     */
    ConstraintUnilateral()
    : d_friction(initData(&d_friction, 0.0, "mu", "Friction"))
    , d_input(initData(&d_input, "input", "Link to detection output"))
    , d_direction(initData(&d_direction, ConstraintDirection(std::bind(&ConstraintUnilateral::defaultGetNormals, this, std::placeholders::_1)), "directions", "Link to detection output")){}

    /*!
     * \brief createConstraintResolution : factory method for constraint solvers
     * \param cst : InternalConstraint
     * \return (UnilateralConstraint|UnilateralFriction)Resolution => abstract ConstraintResolution ptr
     */
    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint * cst) const {
        if (cst->size() == 1) return new UnilateralConstraintResolution();
        else if (cst->size() == 3) return new UnilateralFrictionResolution(d_friction.getValue());
        std::cerr << "Error the size of the constraint is not correct in ConstraintUnilateral" << std::endl;
        return NULL;
    }

    /*!
     * \brief createConstraints
     * processes from and dest geometries using specified algorith;
     * \param[out] constraints : ConstraintContainer
     */
    virtual void createConstraints(ConstraintContainer & constraints) {
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            ConstraintNormal CN = d_direction.getValue().getConstraintNormal(d);

            constraints.push_back(d, CN, std::bind(&ConstraintUnilateral::createConstraintResolution,this,std::placeholders::_1));
        }
    }

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal defaultGetNormals(const collisionAlgorithm::PairDetection & d) {
        defaulttype::Vector3 mainDir = (d.first->getPosition() - d.second->getPosition()).normalized();

    //            defaulttype::Vector3 firstDir = -d.getFirstProximity()->getNormal().normalized();
        defaulttype::Vector3 secondDir = d.second->getNormal().normalized();

        if (mainDir.norm() < std::numeric_limits<double>::epsilon()) mainDir = secondDir;
        if (dot(mainDir,secondDir)<0) mainDir*=-1.0;

        ConstraintNormal CN(mainDir);

        if (d_friction.getValue() != 0.0) CN.addFrame();

        return CN;
    }

};

}

}
