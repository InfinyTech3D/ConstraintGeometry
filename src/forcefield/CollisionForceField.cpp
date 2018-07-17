#include <forcefield/CollisionForceField.h>
#include <stdio.h>
#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <qopengl.h>

namespace constraintGeometry {

CollisionForceField::CollisionForceField()
: p_state("state",RIGHT,this)
, d_stiffness("stiffness",40.0,this)
, p_collision("collision",RIGHT, this)
{}

void CollisionForceField::addForce(VecID f) {
//    std::cout << "ADD FORCE" << std::endl;
    collisionAlgorithm::PairProximityVector & collision = p_collision->getCollisionPairs();

    WriteAccessor<Vector3> force = p_state->write(f);

    for (unsigned i=0;i<collision.size();i++) {
        Vector3 P = collision[i].first->getPosition();
        Vector3 Q = collision[i].second->getPosition();
        Vector3 PQ = P-Q;

        if (p_state() == collision[i].first->getState()) {
            if (dot(PQ,collision[i].second->getNormal())<0) {
                std::map<unsigned,double> line = collision[i].first->getContributions();

                for (std::map<unsigned,double>::iterator it = line.begin();it!=line.end();it++) {
                    force[it->first] -=  PQ * it->second * d_stiffness.getValue();
                }
            }
        } else {
            PQ*=-1;
            if (dot(PQ,collision[i].first->getNormal())<0) {
                std::map<unsigned,double> line = collision[i].second->getContributions();

                for (std::map<unsigned,double>::iterator it = line.begin();it!=line.end();it++) {
                    force[it->first] -=  PQ * it->second * d_stiffness.getValue();
                }
            }
        }
    }
}

void CollisionForceField::addToMatrix(BaseMatrix * /*M*/) {
//    if (d_indice1.getValue() == -1) return;
//    if (d_indice2.getValue() == -1) return;

//    MatrixOffset off1 = M->getOffset(p_state1.get());
//    MatrixOffset off2 = M->getOffset(p_state2.get());

//    unsigned i1 = d_indice1.getValue()*Vector3::size();
//    unsigned i2 = d_indice2.getValue()*Vector3::size();
//    double val = d_stiffness.getValue() * M->getKFactor();

//    M->add(i1+0,i2+0,off1,off2, val);
//    M->add(i1+1,i2+1,off1,off2, val);
//    M->add(i1+2,i2+2,off1,off2, val);

//    M->add(i2+0,i1+0,off2,off1, val);
//    M->add(i2+1,i1+1,off2,off1, val);
//    M->add(i2+2,i1+2,off2,off1, val);

//    M->add(d_indice1.getValue()*Vector3::size()+0,d_indice2.getValue()*Vector3::size()+0,off2,off1,-m_force[0]);
//    M->add(d_indice1.getValue()*Vector3::size()+1,d_indice2.getValue()*Vector3::size()+1,off2,off1,-m_force[1]);
//    M->add(d_indice1.getValue()*Vector3::size()+2,d_indice2.getValue()*Vector3::size()+2,off2,off1,-m_force[2]);

//    M->add(d_indice2.getValue()*Vector3::size()+0,d_indice1.getValue()*Vector3::size()+0,off1,off2,-m_force[0]);
//    M->add(d_indice2.getValue()*Vector3::size()+1,d_indice1.getValue()*Vector3::size()+1,off1,off2,-m_force[1]);
//    M->add(d_indice2.getValue()*Vector3::size()+2,d_indice1.getValue()*Vector3::size()+2,off1,off2,-m_force[2]);
}

void CollisionForceField::draw(const VisualParams *flag) {
    if (!flag->displayFlags().getShowForceFields()) return;
//    if (d_indice1.getValue() == -1) return;
//    if (d_indice2.getValue() == -1) return;

//    Vector3 A = p_state1->get(VecCoordId::position())[d_indice1.getValue()];
//    Vector3 B = p_state2->get(VecCoordId::position())[d_indice2.getValue()];

//    glLineWidth(3);
//    glBegin(GL_LINES);
//    glColor3f(0.8f,0.2f,0.2f); glVertex3dv(A.data());
//    glColor3f(1.0f,0.6f,0.6f); glVertex3dv(B.data());
//    glEnd(); // GL_LINES
//    glLineWidth(1);

    collisionAlgorithm::PairProximityVector & collision = p_collision->getCollisionPairs();
    for (unsigned i=0;i<collision.size();i++) {
        Vector3 P = collision[i].first->getPosition();
        Vector3 Q = collision[i].second->getPosition();


        glLineWidth(3);
        glBegin(GL_LINES);

        if (dot(P-Q,collision[i].second->getNormal())<0) {
            glColor3f(0.8f,0.2f,0.2f); glVertex3dv(P.data());
            glColor3f(1.0f,0.6f,0.6f); glVertex3dv(Q.data());
        }
        glEnd(); // GL_LINES
        glLineWidth(1);
    }
}

DECLARE_CLASS(CollisionForceField)

}

