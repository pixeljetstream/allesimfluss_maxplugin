/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "coretool.h"
#include "brush_flatten.h"

#ifdef SM_SUPPORT_FLATTEN

void SMBrushFlatten::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  if (!m_stroking || SMHit_hasNoFaceHit(hit))
    return;

  int startid = SMHit_getSOid(hit,m_subobj);
  Point3 startpos = hit.face.pos;

  MNMeshConVertBuffer *mcon = m_SM->GetMConVert();

  // compute affected + distances
  mcon->SeedSingle(m_subobj,startid,m_SM->m_brushsize,startpos,TRUE,m_SM->m_ip->SelectionFrozen());
  mcon->BuildAffectedSoft(m_SM->m_brushsize,startpos);
  mcon->AffectedDistWeights(hit.brushinner,hit.brush);

  // run sculpt
  MNMesh* mesh = m_SM->m_mesh;

  Point3  planenormal;
  float planedot;

  if (m_first || !m_fixdist){
    Point3  planepos;
    if(m_SM->m_workplane.IsLocal()){
      mcon->GetAffectedNormalPos(planenormal,planepos,TRUE);
    }
    else{
      planepos = mcon->GetAffectedPos(TRUE);
      planenormal = m_SM->m_workplane.GetNormal();
    }
    m_first = FALSE;
    planedot = DotProd(planenormal,planepos);
    m_planedot = planedot;
    m_planenormal = planenormal;
  }
  else{
    planedot = m_planedot;
    planenormal = m_planenormal;
  }


  IMNMeshUtilities8* mesh8 = static_cast<IMNMeshUtilities8*>(mesh->GetInterface( IMNMESHUTILITIES8_INTERFACE_ID ));
  MNMeshConVertIter it;
  MNMeshConVertIter itend;
  mcon->GetIterators(it,itend);

  float speed = m_speed*hit.pressure;

  MNVert  *vert;
  MESHCONVERT_IT_MASK(mcon->GetMasked())
    vert = &mesh->v[it->vid];
    Point3 projected;
    float dist = ProjectPlanePoint(planenormal,planedot,vert->p,projected);
    vert->p = LERP(vert->p,projected,(speed * it->dist));

  #if MAX_RELEASE >= 9000
    mesh8->InvalidateVertexCache(it->vid);
  #endif
  MESHCONVERT_IT_NORM
    vert = &mesh->v[it->vid];
    Point3 projected;
    float dist = ProjectPlanePoint(planenormal,planedot,vert->p,projected);
    vert->p = LERP(vert->p,projected,(speed * it->dist));
  #if MAX_RELEASE >= 9000
    mesh8->InvalidateVertexCache(it->vid);
  #endif
  MESHCONVERT_IT_END;


  // update draw geo
  MNMesh_updateDrawGeo(mesh,m_SM->m_polyobj);

}

void SMBrushFlatten::StartStroke(int mode,BOOL cont, int subobj, const SMHit &hit)
{

  SM_ASSERT(m_stroking==FALSE);
  SM_ASSERT(subobj==SM_SO_VERTEX || subobj==SM_SO_EDGE || subobj==SM_SO_FACE);

  // undo/redo
  // create restore object
  DbgAssert(!m_actrestore);
  m_actrestore = new MeshVertRestore(m_SM->m_polyobj);

  // and make it active
  DbgAssert(!theHold.Holding());
  if (0 && theHold.Holding()) {
    theHold.Cancel();
  }
  theHold.Begin();

  if (theHold.Holding()){
    theHold.Put (m_actrestore);
  }

  // start the stroke
  m_first = TRUE;
  m_stroking = TRUE;
  m_subobj = subobj;
}

void SMBrushFlatten::EndStroke(const SMHit &hit)
{

  if (m_stroking)
    theHold.Accept(_M("AIF Flatten"));

  m_actrestore = NULL;
  m_stroking = FALSE;

  m_SM->m_epoly->LocalDataChanged(PART_GEOM);
}

void SMBrushFlatten::AbortStroke()
{

  theHold.Cancel();
  m_actrestore = NULL;
  m_stroking = FALSE;

  m_SM->m_epoly->LocalDataChanged(PART_GEOM);
}

#endif

