/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "coretool.h"
#include "brush_tweak.h"

#ifdef SM_SUPPORT_TWEAK

//////////////////////////////////////////////////////////////////////////
// SMStrokeTweakMove

class SMStrokeTweakMove: public SMStroke
{
public:
  SMStrokeTweakMove() {}
  virtual ~SMStrokeTweakMove() {}

  void Init(SketchModeler* oSM, SMBrush *brush) { m_SM = oSM; m_brush = (SMBrushTweak*)brush;}

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  MCHAR* GetUndoMessage() { return _M("AIF Tweak Move");}

private:
  SMBrushTweak*   m_brush;

};


template <int dorun, int doproject>
class MNMeshConVertOpTweak
{
public:
  __forceinline void Apply(MNMeshConVertIter it, const MNMeshConVertBuffer* __restrict mcon){
    int vid = it->vid;
    Point3 pt = dorun ? (orig[vid] + delta * it->dist) : verts[vid].p;
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

  MNMeshConVertOpTweak(const Point3& delta, MNMesh* mesh, const Point3 *orig, WorkPlane& wplane, const Matrix3 &objtm)
    : mesh(mesh),orig(orig),delta(delta),wplane(wplane),objtm(objtm){
    verts = mesh->v;
    objtminv = Inverse(objtm);

#if MAX_RELEASE >= 9000
    mesh8 = static_cast<IMNMeshUtilities8*>(mesh->GetInterface( IMNMESHUTILITIES8_INTERFACE_ID ));
#endif
  }

#if MAX_RELEASE >= 9000
  IMNMeshUtilities8*    mesh8;
#endif
  MNMesh* __restrict    mesh;
  MNVert* __restrict    verts;
  const Point3& delta;
  const Point3* __restrict orig;

  WorkPlane&        wplane;
  Matrix3         objtm;
  Matrix3         objtminv;


};


void SMStrokeTweakMove::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  Matrix3 wmat = m_SM->m_workplane.GetMatrix();
  GraphicsWindow *gw = vpt->getGW();
  Point3 startpos;

  // get delta from start to now
  Matrix3 objectTM = hit.objtm;

  // create ray intersects with workplane, using startpos for reference
  Point3 sposworld = m_brush->m_startwpos;
  Ray curray = hit.wray;
  Point3 curposworld;

  if (!m_SM->m_workplane.Intersect(curray,curposworld,sposworld,true)){
    return;
  }

  if (!m_SM->m_workplane.IsConstrained() && m_SM->m_ip->GetSnapState()){
    IPoint2 sm;
    curposworld = vpt->SnapPoint (m, sm, &wmat);
    curposworld = wmat.PointTransform(curposworld);
  }

  // delta and to local coords
  Point3 delta = curposworld-sposworld;
  delta = Inverse(objectTM).VectorTransform(delta);

  // iterate all affected compute influence based on distance and current brush size
  // use linear falloff for simplicity


  MNMesh  *mesh = m_SM->m_mesh;
  MNMeshConVertBuffer* mcon = m_SM->GetMConVert();

  if (m_SM->m_workplane.HasCollCorrection()){
    MNMeshConVertOpTweak<1,1>op (delta,mesh,m_brush->m_actrestore->undo.Addr(0),m_SM->m_workplane,hit.objtm);
    mcon->ApplyOp<MNMeshConVertOpTweak<1,1>>(op);
  }
  else{
    MNMeshConVertOpTweak<1,0>op (delta,mesh,m_brush->m_actrestore->undo.Addr(0),m_SM->m_workplane,hit.objtm);
    mcon->ApplyOp<MNMeshConVertOpTweak<1,0>>(op);
  }


  MNMesh_updateDrawGeo(mesh,m_SM->m_polyobj);
}

//////////////////////////////////////////////////////////////////////////
// SMStrokeTweakRotateScale
//
// combined to a single stroke
// beause of similarities (need transformcenter)

class SMStrokeTweakEPolyTransform: public SMStroke
{
public:
  SMStrokeTweakEPolyTransform() {}
  virtual ~SMStrokeTweakEPolyTransform() {}

  void Init(SketchModeler* oSM, SMBrush *brush) { m_SM = oSM; m_brush = (SMBrushTweak*)brush;}

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void Start(BOOL cont, int subobj, const SMHit &hit);
  void End(const SMHit &hit);
  void Abort();

  MCHAR* GetUndoMessage() { return _M("AIF Tweak");}

private:
  SMBrushTweak*   m_brush;

  BOOL        m_isflipped;
};


void SMStrokeTweakEPolyTransform::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  Matrix3 wmat = m_SM->m_workplane.GetMatrix();

  Point3 sposworld = m_brush->m_startwpos;
  Point3 curposworld;
  Point2 curmouse = hit.mouse;

  // disable collision (handled by projection)
  if (!m_SM->m_workplane.Intersect(hit.wray,curposworld,sposworld,true)){
    return;
  }

  if (!m_SM->m_workplane.IsConstrained() && m_SM->m_ip->GetSnapState() && m_brush->m_type == SM_TRANSFORM_MOVE){
    IPoint2 sm;
    curposworld = vpt->SnapPoint (m, sm, &wmat);
    curposworld = wmat.PointTransform(curposworld);
    curmouse = Point2((float)sm.x,(float)sm.y);
  }


  float constrscale = m_SM->m_workplane.GetLastConstrainScale();
  Point3 delta = curposworld-sposworld;
  Matrix3 wplaneaxis = m_SM->m_workplane.GetMatrix();
  wplaneaxis.SetRow(3,m_brush->m_center);

  // refill vertices
  m_brush->m_actrestore->Restore(FALSE);

  // scale may need flip
  DWORD flipbits = 0;
  DWORD*  pflipbits = NULL;
  if (m_brush->m_topochange){
    pflipbits = &flipbits;
  }

  Point3 localchange;
  SMTransform_run(m_brush->m_type,m_brush->m_alternative ? 2 : -1,
    m_SM->m_polyobj,delta,(curmouse-m_brush->m_startmouse)*constrscale,localchange,
    wplaneaxis,Matrix3(hit.objtm),vpt->GetVPWorldWidth(sposworld),m_SM->m_ip,pflipbits);
  SMTransform_updateStatus(m_brush->m_type,
    m_SM->m_workplane.GetSwizzled(localchange,!SMTransformType_isScale(m_brush->m_type)),m_SM->m_ip);

  if (m_SM->m_workplane.HasCollCorrection()){
    MNMesh  *mesh = m_SM->m_mesh;
    MNMeshConVertBuffer* mcon = m_SM->GetMConVert();

    MNMeshConVertOpTweak<0,1>op (delta,mesh,m_brush->m_actrestore->undo.Addr(0),m_SM->m_workplane,hit.objtm);
    mcon->ApplyOp<MNMeshConVertOpTweak<0,1>>(op);
  }

  DWORD partchanged = PART_GEOM;

  if (pflipbits){

    BOOL flipped = flipbits; // && !SMFlipEven(flipbits);
    if (flipped != m_isflipped){
      m_SM->m_mesh->FlipElementNormals();
      partchanged |= PART_TOPO;
    }
    m_isflipped = flipped;
  }

  m_SM->m_epoly->LocalDataChanged(partchanged);
  m_SM->m_polyobj->NotifyDependents(FOREVER,partchanged,REFMSG_CHANGE);

}

void SMStrokeTweakEPolyTransform::Start(BOOL cont, int subobj,const SMHit &hit)
{
  m_isflipped = FALSE;

  m_SM->m_polyobj->TransformHoldingStart(m_SM->m_ip->GetTime());

  m_brush->m_center = m_SM->m_ip->GetTransformAxis(m_SM->m_node,0).GetRow(3);

  MNMeshConVertBuffer *mcon = m_SM->GetMConVert();
  // override center in "unselected" case
  if (!mcon->GetSeedSelected() && !m_SM->m_ip->SelectionFrozen()){
    m_brush->m_center = hit.objtm.PointTransform(m_brush->m_startpos);
  }
}

void SMStrokeTweakEPolyTransform::End(const SMHit &hit)
{
  m_SM->m_polyobj->TransformHoldingFinish(m_SM->m_ip->GetTime());
}

void SMStrokeTweakEPolyTransform::Abort()
{

}

//////////////////////////////////////////////////////////////////////////
// SMBrushTweak

void SMBrushTweak::Init(SketchModeler* oSM)
{
  m_SM = oSM;
  m_strokemove = new SMStrokeTweakMove();
  m_strokepoly = new SMStrokeTweakEPolyTransform();

  m_strokemove->Init(oSM,this);
  m_strokepoly->Init(oSM,this);
}


void SMBrushTweak::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{

  Matrix3 wmat = m_SM->m_workplane.GetMatrix();
  GraphicsWindow *gw = vpt->getGW();
  Point3 startpos;

  // get delta from start to now
  Matrix3 objectTM = hit.objtm;

  // create ray intersects with workplane, using startpos for reference
  Point3 sposworld = m_startwpos;
  Ray curray = hit.wray;
  Point3 curposworld;

  if (m_stroke){
    if (m_SM->m_workplane.Intersect(curray,curposworld,sposworld,true) && m_type == SM_TRANSFORM_MOVE)
    {
      if (!m_SM->m_workplane.IsConstrained() && m_SM->m_ip->GetSnapState()){
        IPoint2 sm;
        curposworld = vpt->SnapPoint (m, sm, &wmat);
        curposworld = wmat.PointTransform(curposworld);
      }

      float cursize = m_stroke ? m_strokesize : LERP(hit.brushinner,hit.brush,m_SM->m_usepressure ? hit.pressure : 0);
      m_SM->SetCirclePts(m_SM->m_workplane.GetMatrix(),curposworld,cursize,hit.brush);
      m_SM->m_node->NotifyDependents(FOREVER,PART_TM,REFMSG_CHANGE);
    }
  }
  else{
    if (m_SM->m_workplane.Intersect(curray,curposworld,true))
    {
      float cursize = m_stroke ? m_strokesize : LERP(hit.brushinner,hit.brush,m_SM->m_usepressure ? hit.pressure : 0);
      m_SM->SetCirclePts(m_SM->m_workplane.GetMatrix(),curposworld,cursize,hit.brush);
      m_SM->m_node->NotifyDependents(FOREVER,PART_TM,REFMSG_CHANGE);
    }
  }


  if (m_stroke){
    m_stroke->Proc(vpt,hit,m,lastm);
  }
  else{
    m_SM->CheckTransformDraw(hit);
    m_SM->m_drawtrans = TRUE;
  }

}

void SMBrushTweak::SetConstrained(BOOL state)
{
  if (state == m_SM->m_workplane.IsConstrained()) return;

  if (!state){
    m_SM->m_workplane.DisableConstrained();
  }
  else if (m_stroke){
    m_SM->m_workplane.EnableConstrained(m_startwpos,SMTransformType_isScale(m_type) ? 4 : 2);
  }
}

void SMBrushTweak::SetAlternative(BOOL state)
{
  if (m_SM->m_transformtype == SM_TRANSFORM_ROTATE){
    m_alternative = state;
  }
  else if (SMTransformType_isScale(m_SM->m_transformtype)){
    m_alternative = state;
    m_type = state ? SM_TRANSFORM_SCALE3D : SM_TRANSFORM_SCALE2D;
  }
}


void SMBrushTweak::StartStroke(int mode, BOOL cont, int subobj, const SMHit &hit)
{
  Interface *ip = m_SM->m_ip;

  SM_ASSERT(m_stroke==NULL);
  SM_ASSERT(subobj==SM_SO_VERTEX || subobj==SM_SO_EDGE || subobj==SM_SO_FACE);

  if (SMHit_hasNoFaceHit(hit))
    return;

  m_SM->CheckTransformDraw(hit);

  m_startid = SMHit_getSOid(hit,subobj);
  m_startpos = hit.face.pos;
  m_startwpos = hit.objtm.PointTransform(hit.face.pos);
  m_startmouse = hit.mouse;
  m_alternative = FALSE;
  m_wasactive = FALSE;
  m_strokesize = LERP(hit.brushinner,hit.brush,m_SM->m_usepressure ? hit.pressure : 0);

  if (ip->GetSnapState() && m_type == SM_TRANSFORM_MOVE){
    Matrix3 wmat = m_SM->m_workplane.GetMatrix();

    SM_ViewExp view(ip);
    IPoint2 m = IPoint2((int)hit.mouse.x,(int)hit.mouse.y);

    m_startpos = Inverse(hit.objtm).PointTransform(wmat.PointTransform(view.GetView()->SnapPoint(m,m,&wmat)));
    m_startmouse = Point2((float)m.x,(float)m.y);
  }


  m_type  = (SMTransformType)mode;
  m_subobj = subobj;
  m_SM->m_drawtrans = m_type != SM_TRANSFORM_MOVE;

  MNMeshConVertBuffer *mcon = m_SM->GetMConVert();

  // compute affected + weights
  mcon->SeedSingle(subobj,m_startid,m_SM->m_brushsize,m_startpos,m_soft,m_masked);
  if (m_soft) mcon->BuildAffectedSoft(m_SM->m_brushsize,m_startpos);
  else        mcon->BuildAffectedRigid(false);


  int geoconstrained = FALSE;
  IParamBlock2 *pblock = m_SM->m_epoly->getParamBlock();
  if (pblock){
    pblock->GetValue(ep_constrain_type,ip->GetTime(),geoconstrained,FOREVER);
  }

  // if geo constrain is on or mode > TRANSFORM_MOVE
  // then we need to make use of max's transform system
  // in which case we need to set "lock_select"
  // fill mesh->vertexweights

  if (geoconstrained) // enforce epolytransform
    m_stroke = m_strokepoly;
  else
    m_stroke = m_type == SM_TRANSFORM_MOVE ? (SMStroke*)m_strokemove : (SMStroke*)m_strokepoly;


  // undo/redo
  // create restore object
  // we only need it for "resets" in Transfrom Brush
  // as transform takes care of the real undo
  DbgAssert(!m_actrestore);
  m_actrestore = new MeshVertRestore(m_SM->m_polyobj);


  if (m_stroke == m_strokepoly){
    // polyobj transform

    // sadly transform mode will always try to transform selected fully
    // so we need to deactivate old selection
    SMSOType_getMNMeshSelection(subobj,m_SM->m_mesh,m_oldsel);
    BitArray  temp(SMSOType_getMNMeshCount(subobj,m_SM->m_mesh));


    if (m_soft){
      // we must use regular vertexselectionweights
      // and tell polyobj to use them
      m_oldlocksoft = FALSE;
      m_oldusesel = FALSE;
      if (pblock){
        pblock->GetValue(ep_ss_use,m_SM->m_ip->GetTime(),m_oldusesel,FOREVER);
        pblock->GetValue(ep_ss_lock,m_SM->m_ip->GetTime(),m_oldlocksoft,FOREVER);

        pblock->SetValue(ep_ss_use,m_SM->m_ip->GetTime(),TRUE);
        pblock->SetValue(ep_ss_lock,m_SM->m_ip->GetTime(),TRUE);
      }
      m_SM->m_mesh->SupportVSelectionWeights();
      mcon->AffectedDistWeightsArray(m_strokesize,hit.brush,m_SM->m_mesh->getVSelectionWeights());

      SMSOType_setMNMeshSelection(subobj,m_SM->m_mesh,temp);
    }
    else{
      // rigid mode is ugly, as max's transforms wont work on
      // non-selected stuff, so we need to alter selection
      // temporarily. We cant use softsel, or we loose "clusters"
      if (!m_SM->m_ip->SelectionFrozen() && !mcon->GetSeedSelected()){
        temp.Set(m_startid);
        SMSOType_setMNMeshSelection(subobj,m_SM->m_mesh,temp);
      }
      else if (subobj == SM_SO_FACE){
        // rigid operates only on selected
        // in this case we might allow negative scales

        BitArray  elemfaces(m_SM->m_mesh->FNum());
        m_SM->m_mesh->ElementFromFace(hit.subobj.fid,elemfaces);

        BitArray combined = elemfaces & ~m_oldsel;
        if (combined.IsEmpty() && SMTransformType_isScale(m_type)){
          m_topochange = new TopoChangeRestore(m_SM->m_polyobj);
          m_topochange->Before();
        }
      }
    }

    m_SM->m_polyobj->TransformStart(m_SM->m_ip->GetTime());
  }
  else{
    // our own custom move stroke
    mcon->AffectedDistWeights(m_strokesize,hit.brush);
  }

  if (m_SM->m_workplane.HasCollCorrection()){
    mcon->BuildNormals();
  }


  // and make it active
  DbgAssert(!theHold.Holding());
  if (0 && theHold.Holding()) {
    theHold.Cancel();
  }
  theHold.Begin();

  if (theHold.Holding() && m_stroke != m_strokepoly){
    theHold.Put (m_actrestore);
  }

  // start the stroke
  m_stroke->Start(cont,subobj,hit);
}

void SMBrushTweak::RevertInterimChanges()
{
  if (m_soft){
    IParamBlock2 *pblock = m_SM->m_epoly->getParamBlock();
    if (pblock){
      pblock->SetValue(ep_ss_lock,m_SM->m_ip->GetTime(),m_oldlocksoft);
      pblock->SetValue(ep_ss_use,m_SM->m_ip->GetTime(),m_oldusesel);
    }
  }

  SMSOType_setMNMeshSelection(m_subobj,m_SM->m_mesh,m_oldsel);

  if (m_actrestore){
    delete m_actrestore;
    m_actrestore = NULL;
  }
}

void SMBrushTweak::EndStroke(const SMHit &hit)
{
  /*
  if (SM->m_mtravel < 5){
    AbortStroke();
    m_wasactive = FALSE;
    return;
  }
  */
  m_wasactive = TRUE;
  m_SM->m_drawtrans = TRUE;

  MCHAR* undomessage = _M("AIF Tweak");

  DWORD partchanged = PART_GEOM;

  if (m_stroke){
    undomessage = m_stroke->GetUndoMessage();
    m_stroke->End(hit);

    if (m_topochange){
      m_topochange->After();
      theHold.Put(m_topochange);
      partchanged |= PART_TOPO;
    }
  }

  theHold.Accept(undomessage);

  if (m_stroke == m_strokepoly){
    m_SM->m_polyobj->TransformFinish(m_SM->m_ip->GetTime());
    RevertInterimChanges();
  }

  m_actrestore = NULL;
  m_topochange = NULL;
  m_stroke = NULL;

  m_SM->m_epoly->LocalDataChanged(partchanged);
  m_SM->m_polyobj->NotifyDependents(FOREVER,partchanged,REFMSG_CHANGE);
}

void SMBrushTweak::AbortStroke()
{
  m_SM->m_drawtrans = TRUE;

  if (m_stroke)
    m_stroke->Abort();

  DWORD partchanged = PART_GEOM;

  if (m_topochange){
    m_topochange->After();
    theHold.Put(m_topochange);
    partchanged |= PART_TOPO;
  }

  theHold.Cancel();

  if (m_stroke == m_strokepoly){
    m_SM->m_polyobj->TransformCancel(m_SM->m_ip->GetTime());
    RevertInterimChanges();
  }

  m_actrestore = NULL;
  m_topochange = NULL;
  m_stroke = NULL;


  m_SM->m_epoly->LocalDataChanged(partchanged);
  m_SM->m_polyobj->NotifyDependents(FOREVER,partchanged,REFMSG_CHANGE);
}

SMBrushTweak::~SMBrushTweak()
{
  SM_ASSERT(m_strokemove);
  SM_ASSERT(m_strokepoly);
  delete m_strokemove;
  delete m_strokepoly;
}

BOOL SMBrushTweak::DrawCircleBrush( BOOL meshit )
{
  return meshit && (m_stroke==NULL || m_type == SM_TRANSFORM_MOVE);
}

#endif

