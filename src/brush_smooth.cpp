/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "coretool.h"
#include "brush_smooth.h"

#ifdef SM_SUPPORT_SMOOTH

template <int runop, int doproject>
class MNMeshConVertOpSmooth
{
public:
  __forceinline void Apply(MNMeshConVertIter it, const MNMeshConVertBuffer* __restrict mcon){
    int vid = it->vid;
    Point3  pt = verts[vid].p;

    if (runop){
      float weight = min(it->dist * speed,0.85f);
      pt = LERP(pt,pos,weight);
    }
    if (doproject){
      Point3 ptcorr = pt;
      wplane.CorrectProjection(ptcorr,mcon->GetNormal(it),objtm,objtminv);
      pt = LERP(pt,ptcorr,it->dist);
    }
    verts[vid].p = pt;

#if MAX_RELEASE >= 9000
    mesh8->InvalidateVertexCache(vid);
#endif
  }

  MNMeshConVertOpSmooth(const Point3& pos, MNMesh* mesh, float speed, WorkPlane& wplane, const Matrix3 &objtm)
    : mesh(mesh),speed(speed),pos(pos),wplane(wplane),objtm(objtm){
    objtminv = Inverse(objtm);
    verts = mesh->v;
#if MAX_RELEASE >= 9000
    mesh8 = static_cast<IMNMeshUtilities8*>(mesh->GetInterface( IMNMESHUTILITIES8_INTERFACE_ID ));
#endif
  }

#if MAX_RELEASE >= 9000
  IMNMeshUtilities8*    mesh8;
#endif
  MNMesh*         mesh;
  MNVert* __restrict    verts;
  const Point3      pos;
  float speed;

  WorkPlane&        wplane;
  Matrix3         objtm;
  Matrix3         objtminv;

};

void SMBrushSmooth::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  if (!m_stroking || SMHit_hasNoFaceHit(hit))
    return;

  int startid = SMHit_getSOid(hit,m_subobj);
  Point3 startpos = hit.face.pos;

  MNMeshConVertBuffer *mcon = m_SM->GetMConVert();
  MNMesh* mesh = m_SM->m_mesh;

  // compute affected + distances
  mcon->SeedSingle(m_subobj,startid,m_SM->m_brushsize,startpos,TRUE,m_SM->m_ip->SelectionFrozen(),TRUE,&m_startselected);
  mcon->BuildAffectedSoft(m_SM->m_brushsize,startpos);

  if (m_pinch){
    mcon->AffectedDistWeights(hit.brushinner,hit.brush);
    float speed = hit.pressure*m_speed*0.05f;

    if (m_SM->m_workplane.HasCollCorrection()){
      mcon->BuildNormals();
      MNMeshConVertOpSmooth<1,1> op(hit.face.pos,mesh,speed,m_SM->m_workplane,hit.objtm);
      mcon->ApplyOp<MNMeshConVertOpSmooth<1,1>>(op);
    }
    else{
      MNMeshConVertOpSmooth<1,0> op(hit.face.pos,mesh,speed,m_SM->m_workplane,hit.objtm);
      mcon->ApplyOp<MNMeshConVertOpSmooth<1,0>>(op);
    }


  }
  else{
    mcon->AffectedDistWeightsArray(hit.brushinner,hit.brush);

    // run relax
    MNMeshUtilities   meshutil(mesh);
    meshutil.Relax(0,mcon->GetWeights(),hit.pressure*m_speed,1,false,false,NULL);

    // update draw geo
    if (m_SM->m_workplane.HasCollCorrection()){
      mcon->BuildNormals();
      MNMeshConVertOpSmooth<0,1> op(hit.face.pos,mesh,0.0f,m_SM->m_workplane,hit.objtm);
      mcon->ApplyOp<MNMeshConVertOpSmooth<0,1>>(op);
    }
    else{
#if MAX_RELEASE >= 9000
      MNMeshConVertOpSmooth<0,0> op(hit.face.pos,mesh,0.0f,m_SM->m_workplane,hit.objtm);
      mcon->ApplyOp<MNMeshConVertOpSmooth<0,0>>(op);
#endif
    }
  }


  MNMesh_updateDrawGeo(mesh,m_SM->m_polyobj);

}

void SMBrushSmooth::StartStroke(int mode,BOOL cont, int subobj, const SMHit &hit)
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
  m_stroking = TRUE;
  m_subobj = subobj;

  int startid = SMHit_getSOid(hit,subobj);
  m_startselected = SMSOType_getMNMeshSelected(subobj,m_SM->m_mesh,startid);
}

void SMBrushSmooth::EndStroke(const SMHit &hit)
{

  if (m_stroking)
    theHold.Accept(_M("AIF Smooth"));

  m_actrestore = NULL;
  m_stroking = FALSE;

  m_SM->m_epoly->LocalDataChanged(PART_GEOM);
}

void SMBrushSmooth::AbortStroke()
{

  theHold.Cancel();
  m_actrestore = NULL;
  m_stroking = FALSE;

  m_SM->m_epoly->LocalDataChanged(PART_GEOM);
}

#endif

