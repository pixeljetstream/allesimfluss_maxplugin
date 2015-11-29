/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_MESHRELAX__H
#define __SKETCHMODELER_MESHRELAX__H

#include "maxincludes.h"
#include "helpers.h"
#include "collision.h"

class MNMeshSurfaceRelax 
{
public:
  void Run(const SMRelax &settings);

  MNMeshSurfaceRelax(MNMesh &mesh, const BitArray &faces);
  ~MNMeshSurfaceRelax();

private:
  CollNode                m_collNode;
  CollisionRayTester      m_tester;
  MNMesh                  &m_mesh;
  Tab<int>                m_vertices;
  int                     m_usedVertices;

};



#ifdef SM_SUPPORT_RELAX

//////////////////////////////////////////////////////////////////////////
// Outer Function

void  PolyObject_SurfaceRelax(PolyObject *pobj, const SMRelax &settings, const BitArray &selFaces);
void  PolyObject_SurfaceRelax(PolyObject *pobj, const SMRelax &settings, int startFace);
void  PolyObject_SurfaceRelax(PolyObject *pobj, const SMRelax &settings);

#endif

#endif
