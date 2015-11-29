/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "meshrelax.h"
#include "helpers.h"
#include "restores.h"
#include <algorithm>


#define VERTEX_SMOOTH_FLAG (MN_USER << 1)

MNMeshSurfaceRelax::MNMeshSurfaceRelax( MNMesh &mesh, const BitArray &affectedFaces )
  : m_collNode(mesh,TRUE,TRUE,&affectedFaces)
  , m_mesh(mesh)
{
  // convert faces to interior vertex selection
  m_usedVertices = 0;
  m_vertices.SetCount(m_mesh.numv);
  for (int i = 0; i < m_mesh.numv; i++){
    Tab<int> &faces = m_mesh.vfac[i];
    Tab<int> &edges = m_mesh.vedg[i];
    int facesCount = faces.Count();
    
    int f = 0;
    for (f = 0; f < facesCount && affectedFaces[faces[ f ]] ; f++);


    m_mesh.V(i)->ClearFlag( VERTEX_SMOOTH_FLAG );
    if (f == facesCount){
      int edgesCount = edges.Count();
      int e = 0;
      for (e = 0; e < edgesCount; e++){
        if (m_mesh.E( edges[e] )->f2 < 0)
          break;
      }
      if (e == edgesCount ){
        m_vertices[m_usedVertices++] = i;
        m_mesh.V(i)->SetFlag( VERTEX_SMOOTH_FLAG );
      }
    }
  }
  m_tester.SetCulling(FALSE);
}


MNMeshSurfaceRelax::~MNMeshSurfaceRelax()
{
  m_mesh.ClearVFlags(VERTEX_SMOOTH_FLAG);
}

void MNMeshSurfaceRelax::Run( const SMRelax &settings )
{
  // run the regular relax
  MNMeshUtilities   meshutil(&m_mesh);
  float oneminusweight = 1.0f-settings.weight;
  for (int iter = 0; iter < settings.relaxIters; iter++){
    meshutil.Relax(VERTEX_SMOOTH_FLAG,NULL,settings.relaxAmount,1,true,false,NULL);

    // project affected points
    for (int i = 0; i < m_usedVertices; i++){
      int vert = m_vertices[i];
      Ray ray;
      ray.dir = m_mesh.GetVertexNormal(vert);
      ray.p   = m_mesh.P(vert) - ray.dir * 0.00001f;

      // shoot two rays 
      BOOL hit = m_tester.Collide(ray,m_collNode);
      OpcCollFace_t lastHit = {0};
      if (hit){
        lastHit = m_tester.GetFirstHitFace();
      }
      ray.dir = -ray.dir;
      ray.p   = m_mesh.P(vert) - ray.dir * 0.00001f;
      if (m_tester.Collide(ray,m_collNode)){
        if (!hit || (m_tester.GetFirstHit() < lastHit.mDistance)){
          lastHit = m_tester.GetFirstHitFace();
        }
        hit = TRUE;
      }
      if (hit){
        m_mesh.P(vert) = m_mesh.P(vert) * oneminusweight + m_collNode.GetFacePosition( lastHit, settings.alpha ) * settings.weight;
      }
    }
  }


}



#ifdef SM_SUPPORT_RELAX

//////////////////////////////////////////////////////////////////////////
// Outer Function

void  PolyObject_SurfaceRelax(PolyObject *pobj, const SMRelax &settings, const BitArray& relaxfaces)
{
  // first undo stuff
  if (0 && theHold.Holding()){
    theHold.Cancel();
  }

  theHold.Begin();
  MeshVertRestore *restore = new MeshVertRestore(pobj);

  MNMeshSurfaceRelax relax(pobj->mm,relaxfaces);
  relax.Run(settings);

  // undo
  theHold.Put(restore);

  theHold.Accept(_M("AIF Relax"));

}

void  PolyObject_SurfaceRelax(PolyObject *pobj, const SMRelax &settings, int startFace)
{
  MNMesh  *mesh = &pobj->mm;

  if (startFace < 0 || !mesh->F(startFace)->GetFlag(MN_SEL)){
    return;
  }

  // find selected faces around startface cluster
  MNTempData tmp(mesh);
  MNFaceClusters* clusters = tmp.FaceClusters(MN_SEL);
  int cluster = clusters->clust[startFace];
  BitArray faces(mesh->numf);

  for (int i = 0; i < mesh->numf; i++){
    if (clusters->clust[i] == cluster){
      faces.Set(i);
    }
  }

  PolyObject_SurfaceRelax(pobj,settings,faces);
}

void  PolyObject_SurfaceRelax(PolyObject *pobj, const SMRelax &settings)
{
  MNMesh  *mesh = &pobj->mm;
  BitArray faces;
  mesh->getFaceSel(faces);

  PolyObject_SurfaceRelax(pobj,settings,faces);
}

#endif


