#pragma once

#include <BaseGeometry.h>
#include <Collision.h>

namespace constraintGeometry {

class CollisionForceField :  public ForceField {
public:

    Port<State,REQUIRED> p_state;
    Data<double> d_stiffness;

    Port<collisionAlgorithm::Collision,REQUIRED> p_collision;

    CollisionForceField();

    void addForce(VecID f);

    void addToMatrix(BaseMatrix *M);

    void draw(const VisualParams * );

};

}
