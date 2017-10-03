#ifndef SOFA_COMPONENT_AABBGeometry_INL
#define SOFA_COMPONENT_AABBGeometry_INL

#include "geometry/AABBGeometry.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa {

namespace core {

namespace behavior {

AABBGeometry::AABBGeometry()
: d_nbox(initData(&d_nbox, defaulttype::Vec3i(8,8,8),"nbox","Number of bbox subdivision"))
, d_geometry(initData(&d_geometry, "geometry","Draw Bbox"))
{
    this->f_listening.setValue(true);
}

void AABBGeometry::prepareDetection() {
    if (m_geo == NULL) return;

    m_indexedElement.resize(d_nbox.getValue()[0]+1);
    for (unsigned i=0;i<m_indexedElement.size();i++) {
        m_indexedElement[i].resize(d_nbox.getValue()[1]+1);
        for (unsigned j=0;j<m_indexedElement[i].size();j++) {
            m_indexedElement[i][j].resize(d_nbox.getValue()[2]+1);

            for (unsigned k=0;k<m_indexedElement[i][j].size();k++) {
                m_indexedElement[i][j][k].clear();
            }
        }
    }

    m_Bmin = m_geo->f_bbox.getValue().minBBox();
    m_Bmax = m_geo->f_bbox.getValue().maxBBox();

    m_cellSize[0] = (m_Bmax[0] - m_Bmin[0]) / d_nbox.getValue()[0];
    m_cellSize[1] = (m_Bmax[1] - m_Bmin[1]) / d_nbox.getValue()[1];
    m_cellSize[2] = (m_Bmax[2] - m_Bmin[2]) / d_nbox.getValue()[2];

    // center in -0.5 cellwidth
    m_Bmin -= m_cellSize * 0.5;
    m_Bmax -= m_cellSize * 0.5;


    for (unsigned itE = 0; (int) itE < m_geo->getNbElements(); itE++) {
        helper::vector<defaulttype::Vector3> points;
        m_geo->getElementProximity(itE)->getControlPoints(points);

        if (points.empty()) continue;

        defaulttype::Vector3 minbox = points[0];
        defaulttype::Vector3 maxbox = points[0];

        for (unsigned p=1;p<points.size();p++) {
            minbox[0] = std::min(minbox[0],points[p][0]);
            minbox[1] = std::min(minbox[1],points[p][1]);
            minbox[2] = std::min(minbox[2],points[p][2]);

            maxbox[0] = std::max(maxbox[0],points[p][0]);
            maxbox[1] = std::max(maxbox[1],points[p][1]);
            maxbox[2] = std::max(maxbox[2],points[p][2]);
        }

        defaulttype::Vec3i cminbox;
        defaulttype::Vec3i cmaxbox;

        cminbox[0] = floor((minbox[0] - m_Bmin[0])/m_cellSize[0]);
        cminbox[1] = floor((minbox[1] - m_Bmin[1])/m_cellSize[1]);
        cminbox[2] = floor((minbox[2] - m_Bmin[2])/m_cellSize[2]);

        cmaxbox[0] = ceil((maxbox[0] - m_Bmin[0])/m_cellSize[0]);
        cmaxbox[1] = ceil((maxbox[1] - m_Bmin[1])/m_cellSize[1]);
        cmaxbox[2] = ceil((maxbox[2] - m_Bmin[2])/m_cellSize[2]);

        for (int i=cminbox[0];i<cmaxbox[0];i++) {
            for (int j=cminbox[1];j<cmaxbox[1];j++) {
                for (int k=cminbox[2];k<cmaxbox[2];k++) {
                    m_indexedElement[i][j][k].push_back(itE);
                }
            }
        }
    }
}

void AABBGeometry::init() {
    this->getContext()->get(m_geo,d_geometry.getValue());
    if (m_geo == NULL) serr << "Error cannot find the geometry\n" << sendl;
}

void AABBGeometry::getCloseElements(defaulttype::Vec3i cbox, std::vector<int> & selectElements) {
    int max = std::max(std::max(d_nbox.getValue()[0],d_nbox.getValue()[1]),d_nbox.getValue()[2]);

    int d = 0;

    while (selectElements.empty() && d<max) {
        fillTriangleSet(d,cbox,selectElements);
        d++;// we look for boxed located at d+1
    }
}



void AABBGeometry::getCloseElements(const ConstraintProximityPtr pinfo, std::set<int> & selectElements) {
    helper::vector<defaulttype::Vector3> points;
    pinfo->getControlPoints(points);
    if (points.empty()) return;

    defaulttype::Vector3 minbox = points[0];
    defaulttype::Vector3 maxbox = points[0];
    for (unsigned p=1;p<points.size();p++) {
        minbox[0] = std::min(minbox[0],points[p][0]);
        minbox[1] = std::min(minbox[1],points[p][1]);
        minbox[2] = std::min(minbox[2],points[p][2]);

        maxbox[0] = std::max(maxbox[0],points[p][0]);
        maxbox[1] = std::max(maxbox[1],points[p][1]);
        maxbox[2] = std::max(maxbox[2],points[p][2]);
    }

    defaulttype::Vec3i cminbox;
    defaulttype::Vec3i cmaxbox;

    cminbox[0] = floor((minbox[0] - m_Bmin[0])/m_cellSize[0]);
    cminbox[1] = floor((minbox[1] - m_Bmin[1])/m_cellSize[1]);
    cminbox[2] = floor((minbox[2] - m_Bmin[2])/m_cellSize[2]);

    cmaxbox[0] = ceil((maxbox[0] - m_Bmin[0])/m_cellSize[0]);
    cmaxbox[1] = ceil((maxbox[1] - m_Bmin[1])/m_cellSize[1]);
    cmaxbox[2] = ceil((maxbox[2] - m_Bmin[2])/m_cellSize[2]);

    //project P in the bounding box of the pbject
    //search with the closest box in bbox
    for (int i=0;i<3;i++) {
        if (cminbox[i] < 0) cminbox[i] = 0;
        else if (cminbox[i] > d_nbox.getValue()[i]) cminbox[i] = d_nbox.getValue()[i];

        if (cmaxbox[i] < 0) cmaxbox[i] = 0;
        else if (cmaxbox[i] > d_nbox.getValue()[i]) cmaxbox[i] = d_nbox.getValue()[i];
    }

    for (int i=cminbox[0];i<=cmaxbox[0];i++) {
        for (int j=cminbox[1];j<=cmaxbox[1];j++) {
            for (int k=cminbox[2];k<=cmaxbox[2];k++) {
                defaulttype::Vec3i cbox(i,j,k);
                helper::vector<int> elemntsID;
                getCloseElements(cbox,elemntsID);
                for (unsigned e=0;e<elemntsID.size();e++) selectElements.insert(elemntsID[e]);
            }
        }
    }
}

int AABBGeometry::getNbElements() const {
    return m_geo->getNbElements();
}

ConstraintProximityPtr AABBGeometry::getElementProximity(unsigned eid) const {
    return m_geo->getElementProximity(eid);
}

ElementIteratorPtr AABBGeometry::getBroadPhaseIterator(const ConstraintProximityPtr P) {
    std::set<int> selectElements;
    getCloseElements(P,selectElements);

    return std::make_shared<ElementAABBIterator>(this,selectElements);
}

void AABBGeometry::fillTriangleSet(int d,defaulttype::Vec3i cbox,std::vector<int> & selectElements) {
    for (int i=-d;i<=d;i++) {
        if (cbox[0]+i < 0 || cbox[0]+i > d_nbox.getValue()[0]) continue;

        for (int j=-d;j<=d;j++) {
            if (cbox[1]+j < 0 || cbox[1]+j > d_nbox.getValue()[1]) continue;

            for (int k=-d;k<=d;k++) {
                if (cbox[2]+k < 0 || cbox[2]+k > d_nbox.getValue()[2]) continue;

                if (i!=d && i!=-d && j!=-d && j!=d && k!=-d && k!=d) continue; // already seen

                const helper::vector<unsigned> & elemntsID = m_indexedElement[cbox[0] + i][cbox[1] + j][cbox[2] + k];
                for (unsigned t=0;t<elemntsID.size();t++) selectElements.push_back(elemntsID[t]);
            }
        }
    }
}

void AABBGeometry::draw(const core::visual::VisualParams * vparams) {
    if (!vparams->displayFlags().getShowBoundingCollisionModels()) return;

    for (unsigned i=0;i<m_indexedElement.size();i++) {
        for (unsigned j=0;j<m_indexedElement[i].size();j++) {
            for (unsigned k=0;k<m_indexedElement[i][j].size();k++) {
                if (m_indexedElement[i][j][k].empty()) continue;

                defaulttype::Vector3 points[8];
                points[0] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[1] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[2] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j+1) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[3] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[4] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[5] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j  ) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[6] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[7] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;

                glColor4f(1,1,1,1);
                glBegin(GL_LINES);
                    helper::gl::glVertexT(points[0]);helper::gl::glVertexT(points[1]);
                    helper::gl::glVertexT(points[3]);helper::gl::glVertexT(points[2]);
                    helper::gl::glVertexT(points[7]);helper::gl::glVertexT(points[6]);
                    helper::gl::glVertexT(points[4]);helper::gl::glVertexT(points[5]);

                    helper::gl::glVertexT(points[0]);helper::gl::glVertexT(points[2]);
                    helper::gl::glVertexT(points[1]);helper::gl::glVertexT(points[3]);
                    helper::gl::glVertexT(points[4]);helper::gl::glVertexT(points[6]);
                    helper::gl::glVertexT(points[5]);helper::gl::glVertexT(points[7]);

                    helper::gl::glVertexT(points[0]);helper::gl::glVertexT(points[4]);
                    helper::gl::glVertexT(points[1]);helper::gl::glVertexT(points[5]);
                    helper::gl::glVertexT(points[2]);helper::gl::glVertexT(points[6]);
                    helper::gl::glVertexT(points[3]);helper::gl::glVertexT(points[7]);
                glEnd();
            }
        }
    }
}

} //controller

} //component

}//Sofa

#endif
