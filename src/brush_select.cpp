/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "coretool.h"
#include "brush_select.h"
#include "meshlooping.h"

#ifdef SM_SUPPORT_SELECT

static void updatePaintSelect(SketchModeler* m_SM)
{
  m_SM->m_epoly->LocalDataChanged(PART_SELECT);
  m_SM->m_polyobj->NotifyDependents(FOREVER, PART_SELECT, REFMSG_CHANGE);
  //IMeshSelect* ms = (m_SM->m_polyobj) ? GetMeshSelectInterface(m_SM->m_polyobj) : NULL;
  //if (ms) ms->LocalDataChanged();
  m_SM->m_node->EvalWorldState(m_SM->m_ip->GetTime());
}


//////////////////////////////////////////////////////////////////////////
// SMStrokeSelectPaint

#define STROKESELECTPAINT_MAX_TOUCHED 8
#define STROKESELECTPAINT_MAX_ADVANCES  8

class SMStrokeSelectPaint : public SMStroke
{
public:
  SMStrokeSelectPaint() {}
  virtual ~SMStrokeSelectPaint() {}

  void Init(SketchModeler* oSM, SMBrush *brush) {m_SM = oSM; m_brush = (SMBrushSelect*) brush;}

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void Start(BOOL cont, int subobj, const SMHit &hit);
  void End(const SMHit &hit);
  MCHAR* GetUndoMessage() { return _M("AIF Select Paint");}

private:
  BOOL RunConnect(MNMeshLoopAdvancer &adv, int curid, Point3 &m, GraphicsWindow *gw, bool selstate);
  void  Paint(BOOL addcur, ViewExp *vpt, const SMHit &hit, BOOL addfailed);


  SMBrushSelect*  m_brush;
  int       m_subobjmode;

  Point3      m_lastmouse;
  int       m_lastid;
  int       m_lastcon;

  MNMeshLoopItem  m_items[STROKESELECTPAINT_MAX_ADVANCES];

  int       m_touched[STROKESELECTPAINT_MAX_TOUCHED];
  int       m_numtouched;
};


BOOL  SMStrokeSelectPaint::RunConnect(MNMeshLoopAdvancer &adv, int curid, Point3 &m, GraphicsWindow *gw, bool selstate)
{

  if (m_lastid < 0){
    adv.Select(curid,selstate);
    m_lastmouse = m;
    m_lastid = curid;
    m_numtouched = 0;
    return TRUE;
  }


  int       conid;
  // first check if we are connected, if yes, then add to touch list
  if (adv.AreConnected(m_lastid,curid,conid)){
    m_touched[(m_numtouched++)%STROKESELECTPAINT_MAX_TOUCHED] = curid;
    return TRUE;
  }

  // we are not connected directly, check for touching list
  int check = min(m_numtouched,STROKESELECTPAINT_MAX_TOUCHED);
  for (int i = 0; i < check; i++){
    int touch = m_touched[i];
    if (adv.AreConnected(touch,curid,conid)){
      adv.Select(touch,selstate);
      m_lastmouse = m;
      m_lastid = touch;
      m_numtouched = 0;
      return TRUE;
    }
  }

  // neither touch nor directly, check for loop
  // stop loop 1 before currentid
  if(TRUE){
    MNMeshLoopItem  *browse = m_items;
    Tab<MNMeshLoopFront>  fronts;
    MNMeshLoopFront   *front;

    int outcount = adv.SetupFront(m_lastid,fronts);
    if (!outcount)
      return FALSE;

    Point3 start;
    Point3 test = adv.GetPos(m_lastid);
    gw->transPoint(&test,&start); start.z = 0.0f;
    test = (m-m_lastmouse).Normalize();

    front = adv.FindDirectedFront(fronts,start, test, gw);
    if (!front)
      return FALSE;

    browse->id = front->previd;
    browse->prev = NULL;
    browse++;

    for (int s = 1; s < STROKESELECTPAINT_MAX_ADVANCES; s++,browse++){
      if (front->nextid == curid || adv.Advance(front,browse))
        break;

      browse->prev = browse-1;
    }

    if (front->nextid != curid)
      return FALSE;
    // move browse back to last valid
    browse--;

    // we found a connection, end here
    m_lastmouse = m;
    m_lastid = browse->id;
    m_numtouched = 0;

    while (browse){
      adv.Select(browse->id,selstate);
      browse = browse->prev;
    }

    return TRUE;
  }

  return FALSE;
}

void  SMStrokeSelectPaint::Paint(BOOL addcur, ViewExp *vpt, const SMHit &hit, BOOL addfailed)
{
  int curid = SMHit_getSOid(hit,m_subobjmode);
  if (SMHit_hasNoFaceHit(hit) || curid == m_lastid)
    return;

  GraphicsWindow *gw = vpt->getGW();
  gw->setTransform(m_SM->m_node->GetObjectTM(m_SM->m_ip->GetTime()));

  Point3 mp;
  gw->transPoint(&hit.face.pos,&mp);

  bool additive = m_brush->m_additive ? true : false;

  // first selected is a quickie
  switch(m_subobjmode){
  case SM_SO_VERTEX:
    {
      MNMeshLoopAdvancerVertex adv(m_SM->m_mesh);
      BOOL con = RunConnect(adv,curid,mp,gw,additive);
      if ((addcur && con) || (addfailed && !con)){
        adv.Select(curid,additive);
        m_lastid = curid;
        m_lastmouse = mp;
        m_numtouched = 0;
      }
    }
    break;
  case SM_SO_EDGE:
    {
      // disable ring selection for edges
      MNMeshLoopAdvancerEdge adv(m_SM->m_mesh,~3);
      BOOL con = RunConnect(adv,curid,mp,gw,additive);
      if ((addcur && con) || (addfailed && !con)){
        adv.Select(curid,additive);
        m_lastid = curid;
        m_lastmouse = mp;
        m_numtouched = 0;
      }
    }
    break;
  case SM_SO_FACE:
    {
      MNMeshLoopAdvancerFace adv(m_SM->m_mesh);
      BOOL con = RunConnect(adv,curid,mp,gw,additive);
      if ((addcur && con) || (addfailed && !con)){
        adv.Select(curid,additive);
        m_lastid = curid;
        m_lastmouse = mp;
        m_numtouched = 0;
      }
    }
    break;
  case SM_SO_BORDER:
    {
      BitArray elements(m_SM->m_mesh->ENum());

      m_SM->m_mesh->BorderFromEdge(curid,elements);
#if 0
      BitArray cursel(SM->m_mesh->ENum());
      SM->m_mesh->getEdgeSel(cursel);

      if (additive){
        cursel |= elements;
      }
      else{
        cursel &= ~elements;
      }

      SM->m_mesh->EdgeSelect(cursel);
#else
      BitArrayEdgeSelection cb(m_SM->m_mesh,additive);
      SM_BITARRAY_ENUMSET(elements,cb,BitArrayEdgeSelection);
#endif
    }
    break;
  case SM_SO_ELEMENT:
    {
      BitArray elements;
      elements.SetSize(m_SM->m_mesh->FNum());

      m_SM->m_mesh->ElementFromFace(curid,elements);

#if 0
      BitArray cursel(SM->m_mesh->FNum());
      SM->m_mesh->getFaceSel(cursel);

      if (additive){
        cursel |= elements;
      }
      else{
        cursel &= ~elements;
      }

      SM->m_mesh->FaceSelect(cursel);
#else
      BitArrayFaceSelection cb(m_SM->m_mesh,additive);
      SM_BITARRAY_ENUMSET(elements,cb,BitArrayFaceSelection);
#endif
    }
    break;

  }

  updatePaintSelect(m_SM);
}

void  SMStrokeSelectPaint::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  int subobjbase = SMSOTypeToBase(m_subobjmode);
  Paint(subobjbase != SM_SO_EDGE,vpt,hit,subobjbase != SM_SO_EDGE);
}

void SMStrokeSelectPaint::Start(BOOL cont, int subobj, const SMHit &hit) {
  m_subobjmode = subobj;
  m_lastid = -1;
  m_lastcon = -1;
  m_numtouched = 0;
}
void SMStrokeSelectPaint::End(const SMHit &hit)
{
  Interface *ip = m_SM->m_ip;
  SM_ViewExp view(ip);
  ViewExp *vpt = view.GetView();

  Paint(TRUE, vpt, hit, FALSE);
  
  updatePaintSelect(m_SM);
}


//////////////////////////////////////////////////////////////////////////
// SMStrokeSelectLoopPart

class SMStrokeSelectLoopPart : public SMStroke
{
public:
  SMStrokeSelectLoopPart() : m_lastclosest(NULL){}
  virtual ~SMStrokeSelectLoopPart() {}

  void Init(SketchModeler* oSM, SMBrush *brush) {m_SM = oSM; m_brush = (SMBrushSelect*) brush;}

  void Display(TimeValue t, ViewExp *vpt, int flags);
  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void Start(BOOL cont, int subobj, const SMHit &hit);
  void End(const SMHit &hit);
  void Abort(){ Clear(); m_lastid = -1;}

  MCHAR* GetUndoMessage() { return _M("AIF Select Partial Loop");}

private:
  void Clear();

  MNMeshLoopItem*   FindClosestLoopItem(GraphicsWindow *gw, const SMHit &hit);

  SMBrushSelect*    m_brush;
  int         m_subobjmode;
  MNMeshLoopItem*   m_lastclosest;
  int         m_lastid;

  MNMeshLoop      m_loop;
  BitArray      m_oldselection;

};

void SMStrokeSelectLoopPart::Display(TimeValue t, ViewExp *vpt, int flags)
{
  if (!m_lastclosest)
    return;

  INode*      node = m_SM->m_node;
  MNMesh*     mesh = m_SM->m_mesh;

  GraphicsWindow *gw = vpt->getGW();
  // zbuffered ?
  int savedLimits = gw->getRndLimits();
  int newLimits = savedLimits;
  if  (FALSE) newLimits |= GW_Z_BUFFER;
  else    newLimits &= ~GW_Z_BUFFER;
  gw->setRndLimits((newLimits & ~GW_ILLUM) & ~GW_TEXTURE);

  // obj transforms
  Matrix3 mat = node->GetObjectTM(t);
  Matrix3 viewmat;
  vpt->GetAffineTM(viewmat);
  viewmat = Inverse(viewmat);
  Point3 viewdir = viewmat.GetRow(2);
  gw->setTransform(mat);
  mat = Inverse(mat);
  viewdir = mat.VectorTransform(viewdir);

  // colors
  gw->setColor(LINE_COLOR,m_SM->m_hwmat.Kd);
  gw->setColor(FILL_COLOR,m_SM->m_hwmat.Kd);

  MNMeshLoopItem* browse = m_lastclosest;

  // full draw
  switch(m_subobjmode)
  {
  case SM_SO_VERTEX:
    while(browse ){
      gw->marker(&mesh->P(browse->id),CIRCLE_MRKR);
      browse = browse->prev;
    }
    break;
  case SM_SO_EDGE:
    {
      Point3 pts[3];
      while(browse ){
        MNEdge *edge = mesh->E(browse->id);
        MNMesh_renderMNEdge(gw,mesh,edge,pts);
        browse = browse->prev;
      }
    }
    break;
  case SM_SO_FACE:
    {

      Point3 pts[3];
      Point3 clrs[3];
      Tab<int> triVerts;

      gw->startTriangles();
      while(browse ){
        Point3 normal = mesh->GetFaceNormal(browse->id,TRUE);
        float dot = DotProd(normal,viewdir);
        Point3 clr = m_SM->m_hwmat.Kd*(max(dot,0)) + ((1.f-m_SM->m_hwmat.Kd)*(max(-dot,0)));
        MNFace *face = mesh->F(browse->id);
        MNMesh_renderMNFace(gw,mesh,face,triVerts,pts,clrs,Color(clr));
        browse = browse->prev;
      }
      gw->endTriangles();

    }
    break;
  default:
    break;
  }
  // line draw
  /*
  while(browse && browse->prev){

    browse = browse->prev;
  }
  */

  gw->setRndLimits(savedLimits);

}

void SMStrokeSelectLoopPart::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  MNMesh    *mesh = m_SM->m_mesh;
  if (!m_loop.m_numitems)
    return;

  //Matrix3 vmat;
  //vpt->GetAffineTM(vmat);
  //Point3 vdir = Inverse(vmat).GetRow(2);

  Matrix3 mat = m_SM->m_node->GetObjectTM(m_SM->m_ip->GetTime());
  GraphicsWindow *gw = vpt->getGW();
  gw->setTransform(mat);
  //vdir = Inverse(mat).VectorTransform(vdir);

  m_lastclosest = FindClosestLoopItem(gw,m_SM->m_hit);
}

#define LOOP_PART_LASTMIN 8

MNMeshLoopItem* SMStrokeSelectLoopPart::FindClosestLoopItem(GraphicsWindow *gw, const SMHit &hit)
{
  float mindist = 10000000.0f;
  int   minwave = 10000000;
  float dist;
  MNMeshLoopItem* closest = NULL;
  MNMeshLoopItem* browse = m_loop.m_items.Addr(0);
  MNMeshLoopItem* end = browse+m_loop.m_numitems;

  Point3 whit;
  Point3 test;

  MNMeshLoopItem* lastmin[LOOP_PART_LASTMIN];
  int numlastmin = 0;

  //whit = *hit - (DotProd(planenormal,*hit)*planenormal);

  gw->transPoint(&hit.face.pos,&whit);
  whit.z = 0.0f;

  // find closest wave
  for(;browse < end; browse++){
    gw->transPoint(&browse->pos,&test);
    test.z = 0.0f;
    dist = (test-whit).LengthSquared();

    browse->distance = dist;
    if (dist < mindist){
      lastmin[(numlastmin++)%LOOP_PART_LASTMIN] = browse;
      mindist = dist;
      closest = browse;
    }
  }

  int curid = SMHit_getSOid(hit,m_subobjmode);

  int check = min(numlastmin,LOOP_PART_LASTMIN);
  for (int i = 1; i < check; i++){
    browse = lastmin[i];
    if (browse->distance < 1000 && (browse->wave < closest->wave || browse->id == curid)){
      closest = browse;
    }
  }

  return closest;
}

void SMStrokeSelectLoopPart::Start(BOOL cont, int subobj, const SMHit &hit) {
  m_subobjmode = SMSOTypeToBase(subobj);
  m_lastclosest = NULL;

  if (!cont){
    m_lastid = -1;
    m_SM->SetHitPush(hit);
  }
  else{
    SMHit newhit = hit;
    SMHit_setSOid(newhit,subobj,m_lastid);
    m_SM->SetHitPush(newhit);
  }


  MNMesh    *mesh = m_SM->m_mesh;
  int     startid;


  // grab old selection
  // select loops
  // build loopitems and select them

  switch(subobj){
  case SM_SO_VERTEX:
  {
    BitArray  subsel(mesh->VNum());
    MNMeshLoopAdvancerVertex advancer(mesh);

    startid = m_lastid < 0 ? hit.subobj.vid : m_lastid;

    m_loop.Build(advancer,startid,subsel);
    m_loop.LinkPrev();

  }
    break;
  case SM_SO_EDGE:
  {
    BitArray  subsel(mesh->ENum());
    MNMeshLoopAdvancerEdge advancer(mesh);

    startid = m_lastid < 0 ? m_SM->m_hit.subobj.eid : m_lastid;

    m_loop.Build(advancer,startid,subsel);
    m_loop.LinkPrev();

  }
    break;
  case SM_SO_FACE:
  {
    BitArray  subsel(mesh->FNum());
    MNMeshLoopAdvancerFace advancer(mesh);

    startid = m_lastid < 0 ? m_SM->m_hit.subobj.fid : m_lastid;

    m_loop.Build(advancer,startid,subsel);
    m_loop.LinkPrev();

  }
    break;
  }
  
}

void SMStrokeSelectLoopPart::Clear()
{
  m_lastclosest = NULL;
  m_loop.Clear();

  m_oldselection.SetSize(0);
}

void SMStrokeSelectLoopPart::End(const SMHit &hit)
{
  // load old, then apply new
  // walk down subobjs
  MNMesh    *mesh = m_SM->m_mesh;

  MNMeshLoopItem* browse = m_lastclosest;

  bool additive = m_brush->m_additive ? true : false;

  if(m_lastclosest){
    switch(m_subobjmode)
    {
    case SM_SO_VERTEX:
      while(browse){
        mesh->V(browse->id)->SetFlag(MN_SEL,additive);
        browse = browse->prev;
      }
      break;
    case SM_SO_EDGE:
      while(browse){
        mesh->SetEdgeSel(browse->id,additive ? 1 : 0);
        browse = browse->prev;
      }
      break;
    case SM_SO_FACE:
      while(browse){
        mesh->F(browse->id)->SetFlag(MN_SEL,additive);
        browse = browse->prev;
      }
      break;
    }
    m_lastid = m_lastclosest->id;
  }

  updatePaintSelect(m_SM);

  Clear();
}

//////////////////////////////////////////////////////////////////////////
// SMStrokeSelectLoopFull

class SMStrokeSelectLoopFull: public SMStroke
{
public:
  SMStrokeSelectLoopFull() {}
  virtual ~SMStrokeSelectLoopFull() {}

  void Init(SketchModeler* oSM, SMBrush *brush) {m_SM = oSM; m_brush = (SMBrushSelect*) brush;}

  void Start(BOOL cont, int subobj, const SMHit &hit);
  void End(const SMHit &hit);
  void Abort(){}
  MCHAR* GetUndoMessage() { return _M("AIF Select Full Loop");}

private:
  void AdvanceAndSelect(MNMeshLoopAdvancer &adv, GraphicsWindow *gw, Point3 &mdir, BOOL seladditive);

  SMBrushSelect*    m_brush;
  int         m_subobjmode;

  int         m_startid;
  Point3        m_startpos;
};

void SMStrokeSelectLoopFull::Start(BOOL cont, int subobj, const SMHit &hit)
{
  m_SM->SetHitPush(hit);

  m_subobjmode = SMSOTypeToBase(subobj);

  m_startid = SMHit_getSOid(hit,subobj);
  m_startpos = hit.face.pos;
}

void SMStrokeSelectLoopFull::AdvanceAndSelect(MNMeshLoopAdvancer &adv, GraphicsWindow *gw, Point3 &mdir, BOOL seladditive)
{
  MNMeshLoopItem    item;
  Tab<MNMeshLoopFront>  fronts;
  MNMeshLoopFront   *front;

  BitArray finalsel(adv.GetCount());

  finalsel.Set(m_startid);

  int outcount = adv.SetupFront(m_startid,fronts);
  if (outcount){

    Point3 start;
    Point3 test = adv.GetPos(m_startid);
    gw->transPoint(&test,&start); start.z = 0.0f;


    front = adv.FindDirectedFront(fronts,start, mdir, gw);
    if (front){
      // further advance the outloop
      finalsel.Set(front->previd);

      while (TRUE){
        if (adv.Advance(front,&item,0,1,finalsel))
          break;
      }
    }
  }

  BitArray cursel(adv.GetCount());
  adv.GetSelected(cursel);
  if (seladditive)  cursel |= finalsel;
  else        cursel &= ~finalsel;
  adv.SetSelected(cursel);
}

void SMStrokeSelectLoopFull::End(const SMHit &hit)
{
  Interface *ip = m_SM->m_ip;

  if (m_startid < 0)
    return;

  SM_ViewExp view(ip);

  ViewExp *actvpt = view.GetView();
  GraphicsWindow *gw = actvpt->getGW();

  Point3 mstartpos;
  Point3 mendpos;

  gw->setTransform(m_SM->m_node->GetObjectTM(m_SM->m_ip->GetTime()));
  gw->transPoint(&m_startpos,&mstartpos); mstartpos.z = 0.0f;

  mendpos = Point3(hit.mouse.x,hit.mouse.y,0.0f);

  Point3 mdir = (mendpos-mstartpos).Normalize();
  MNMesh *mesh = m_SM->m_mesh;

  switch(m_subobjmode)
  {
  case SM_SO_VERTEX:
  {
    MNMeshLoopAdvancerVertex adv(mesh);
    AdvanceAndSelect(adv,gw,mdir,m_brush->m_additive);
  }
    break;
  case SM_SO_EDGE:
  {
    MNMeshLoopAdvancerEdge adv(mesh);
    AdvanceAndSelect(adv,gw,mdir,m_brush->m_additive);
  }
    break;
  case SM_SO_FACE:
  {
    MNMeshLoopAdvancerFace adv(mesh);
    AdvanceAndSelect(adv,gw,mdir,m_brush->m_additive);
  }
    break;
  default:
    break;
  }

  updatePaintSelect(m_SM);
}

//////////////////////////////////////////////////////////////////////////
// SMStrokeSelectShape

class SMShape
{
public:
  virtual int Contains(const IPoint2 &pt) = 0;
};

class SMShapeRect : public SMShape
{
public:
  Rect  m_rect;
  SMShapeRect(const Rect &rect) : m_rect(rect) {}
  int Contains(const IPoint2 &pt) {
    return m_rect.Contains(pt);
  }
};

class SMShapeCircle : public SMShape
{
public:
  IPoint2 m_center;
  int   m_rad;

  SMShapeCircle(const IPoint2 &ctr, int rad) : m_center(ctr),m_rad(rad) {}
  int Contains(const IPoint2 &pt) {
    return Length(pt-m_center) < m_rad;
  }
};

class SMShapePolygon : public SMShape
{
public:
  std::vector<IPoint2> m_pts;
  int          m_cnt;

  SMShapePolygon(const std::vector<IPoint2> &pts) : m_pts(pts),m_cnt((int)m_pts.size()) {}
  virtual ~SMShapePolygon() {}

  int Contains(const IPoint2 &pt) {
    int  oddNodes = FALSE;
    for (int i=0, j = m_cnt-1; i < m_cnt; i++) {
      IPoint2 &ai = m_pts[i];
      IPoint2 &bi = m_pts[j];

      Point2  a(float(ai.x),float(ai.y));
      Point2  b(float(bi.x),float(bi.y));

      float x = float(pt.x);
      float y = float(pt.y);

      if (  a.y < y && b.y >= y
        ||  b.y < y && a.y >= y)
      {
        if ((a.x +(y-a.y)/(b.y-a.y)*(b.x-a.x )) < x ) {
          oddNodes = !oddNodes;
        }
      }
      j = i;
    }

    return oddNodes;
  }
};

class SMStrokeSelectShape: public SMStroke
{
public:
  SMStrokeSelectShape() {}
  virtual ~SMStrokeSelectShape() {}

  void Init(SketchModeler* oSM, SMBrush *brush) {m_SM = oSM; m_brush = (SMBrushSelect*) brush;}

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void Start(BOOL cont, int subobj, const SMHit &hit);
  void End(const SMHit &hit);
  void Abort();
  MCHAR* GetUndoMessage() { return _M("AIF Select Shape");}

private:
  void Fill(GraphicsWindow *gw, BitArray &barray, DWORD kickflag, SMShape &shape);

  SMBrushSelect*    m_brush;
  int         m_subobjmode;

  int         m_hittype;

  BOOL          m_additive;
  BitArray        m_oldsel;
  std::vector<IPoint2>  m_polypoints;
  IPoint2         m_startpoint;
  IPoint2         m_lastpoint;
  BOOL          m_drawactive;
  HWND          m_hwnd;
};

void SMStrokeSelectShape::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  // draw rect
  m_hwnd = vpt->GetHWnd();

  SMDrawSelShape(m_hwnd,m_hittype,m_drawactive,TRUE,m_startpoint,m,lastm,m_polypoints);

  m_lastpoint = m;
  m_drawactive = TRUE;
}

void SMStrokeSelectShape::Start(BOOL cont, int subobj, const SMHit &hit)
{
  // custom redraw
  m_SM->m_redrawviews = FALSE;
  m_drawactive = FALSE;
  m_startpoint = IPoint2(int(hit.mouse.x),int(hit.mouse.y));
  m_subobjmode = subobj;

  m_hittype = getHitType();
  switch(m_hittype)
  {
  case HITTYPE_CIRCLE:
  case HITTYPE_BOX:
    break;
  case HITTYPE_LASSO:
    m_polypoints.push_back(m_startpoint);
    m_polypoints.push_back(m_startpoint);
    break;
  default:
    m_hittype = HITTYPE_BOX;
    break;
  }
}
void SMStrokeSelectShape::Abort()
{
  SMDrawSelShape(m_hwnd,m_hittype,m_drawactive,FALSE,m_startpoint,m_lastpoint,m_lastpoint,m_polypoints);

  m_SM->m_fullredraw = TRUE;
  m_SM->m_redrawviews = TRUE;
  m_drawactive = FALSE;
  m_polypoints.clear();
}

void SMStrokeSelectShape::Fill(GraphicsWindow *gw, BitArray &insideverts, DWORD kickflag, SMShape &shape)
{
  MNVert  *vert = m_SM->m_mesh->v;
  int vcnt = m_SM->m_mesh->VNum();
  for (int i = 0; i < vcnt; i++,vert++){
    if (vert->GetFlag(kickflag))
      continue;

    IPoint3 pt;
    gw->wTransPoint(&vert->p,&pt);
    if(shape.Contains(IPoint2(pt.x,pt.y)))
      insideverts.Set(i,true);
  }
}

void SMStrokeSelectShape::End(const SMHit &hit)
{

  // run selection process
  GraphicsWindow  *gw = SM_ViewExp(m_SM->m_ip).GetView()->getGW();

  int ignorebackfacing = FALSE;
  IParamBlock2 *pblock = m_SM->m_epoly->getParamBlock();
  if (pblock){
    pblock->GetValue(ep_ignore_backfacing,m_SM->m_ip->GetTime(),ignorebackfacing,FOREVER);
  }

  MNMesh  *mesh = m_SM->m_mesh;
  if (ignorebackfacing){
    mesh->UpdateBackfacing(gw,false);
  }


  // flag all vertices inside the rectangle
  int vcnt = mesh->VNum();
  BitArray  insideverts(vcnt);
  DWORD kickflag = MN_DEAD | (ignorebackfacing ? MN_BACKFACING : 0);

  switch(m_hittype)
  {
  case HITTYPE_LASSO:
    {
      SMShapePolygon shape(m_polypoints);
      Fill(gw,insideverts,kickflag,shape);
    }
    break;
  case HITTYPE_CIRCLE:
    {
      IPoint2 diff = m_lastpoint-m_startpoint;
      int rad = max(abs(diff.x),abs(diff.y));

      SMShapeCircle shape(m_startpoint,rad);
      Fill(gw,insideverts,kickflag,shape);
    }
    break;
  case HITTYPE_BOX:
  default:
    {
      Rect rect;
      rect += m_startpoint;
      rect += m_lastpoint;

      SMShapeRect shape(rect);
      Fill(gw,insideverts,kickflag,shape);
    }
    break;
  }

  MNMesh_selectBasedOnVerts(mesh,m_subobjmode,kickflag,insideverts,m_SM->m_ip->GetCrossing(),m_brush->m_additive ? true : false);

  updatePaintSelect(m_SM);

  // for resets
  Abort();
}
//////////////////////////////////////////////////////////////////////////
//

class SMStrokeSelectPaintWide: public SMStroke
{
public:
  SMStrokeSelectPaintWide() {}
  virtual ~SMStrokeSelectPaintWide() {}

  void Init(SketchModeler* oSM, SMBrush *brush) {m_SM = oSM; m_brush = (SMBrushSelect*) brush;}

  void Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm);

  void Start(BOOL cont, int subobj, const SMHit &hit);
  MCHAR* GetUndoMessage() { return _M("AIF Select Paint Wide");}

private:
  SMBrushSelect*    m_brush;
  int         m_subobjmode;

  BOOL        m_ignorebackfacing;
};


void SMStrokeSelectPaintWide::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  if (SMHit_hasNoFaceHit(hit))
    return;

  int startid = SMHit_getSOid(hit,m_subobjmode);
  Point3 startpos = hit.face.pos;

  MNMeshConVertBuffer *mcon = m_SM->GetMConVert();
  MNMesh* mesh = m_SM->m_mesh;

  // compute affected
  int sobase = SMSOTypeToBase(m_subobjmode);
  float maxdist = LERP(hit.brushinner,hit.brush,hit.pressure);
  mcon->SeedSingle(sobase,startid,maxdist,startpos,TRUE,FALSE);
  mcon->BuildAffectedSoft(maxdist,startpos,false);


  //BitArray insideverts(mesh->VNum());
  //mcon->AffectedToBitArray(insideverts);

  BitArray &insideverts = mcon->GetUntouched();
  insideverts = insideverts.operator ~();

  // test seed against size

  if (m_ignorebackfacing){
    mesh->UpdateBackfacing(vpt->getGW(),false);
  }

  bool additive = m_brush->m_additive ? true : false;
  DWORD kickflag = MN_HIDDEN | MN_DEAD | (m_ignorebackfacing ? MN_BACKFACING : 0);
  MNMesh_selectBasedOnVerts(mesh,sobase,kickflag,insideverts,m_SM->m_ip->GetCrossing(),additive);

  // always select current hit
  switch (sobase){
  case SM_SO_VERTEX:
    mesh->V(startid)->SetFlag(MN_SEL,additive); 
    break;
  case SM_SO_EDGE:
    mesh->SetEdgeSel(startid,additive ? 1 : 0);
    break;
  case SM_SO_FACE:
    mesh->F(startid)->SetFlag(MN_SEL,additive);
    break;
  }
  

  updatePaintSelect(m_SM);
}

void SMStrokeSelectPaintWide::Start(BOOL cont, int subobj, const SMHit &hit)
{
  m_SM->GetMConVert()->CheckMesh(m_SM->m_mesh);
  m_subobjmode = subobj;
  m_ignorebackfacing = FALSE;
  IParamBlock2 *pblock = m_SM->m_epoly->getParamBlock();
  if (pblock){
    pblock->GetValue(ep_ignore_backfacing,m_SM->m_ip->GetTime(),m_ignorebackfacing,FOREVER);
  }
}

//////////////////////////////////////////////////////////////////////////
// SMBrushSelect

class SMStrokeSelectPacket{
public:
  SMStrokeSelectPaint theStrokeSelectPaint;
  SMStrokeSelectLoopPart theStrokeSelectLoopPart;
  SMStrokeSelectLoopFull theStrokeSelectLoopFull;
  SMStrokeSelectShape theStrokeSelectShape;
  SMStrokeSelectPaintWide theStrokeSelectPaintWide;
};

void SMBrushSelect::Init(SketchModeler* oSM)
{
  m_SM = oSM;

  m_strokepacket = new SMStrokeSelectPacket();
  SM_ASSERT(m_strokepacket);

  m_strokes[SM_SELECT_PAINT] = &m_strokepacket->theStrokeSelectPaint;
  m_strokes[SM_SELECT_LOOPPART] = &m_strokepacket->theStrokeSelectLoopPart;
  m_strokes[SM_SELECT_LOOPFULL] = &m_strokepacket->theStrokeSelectLoopFull;
  m_strokes[SM_SELECT_SHAPE] = &m_strokepacket->theStrokeSelectShape;
  m_strokes[SM_SELECT_PAINTWIDE] = &m_strokepacket->theStrokeSelectPaintWide;

  for (int i = 0; i < SM_SELECTS; i++)
    m_strokes[i]->Init(oSM,this);
}

void SMBrushSelect::Display(TimeValue t, ViewExp *vpt, int flags)
{
  if (m_stroke){
    m_stroke->Display(t,vpt,flags);
    return;
  }
}

void SMBrushSelect::GetViewportRect( TimeValue t, ViewExp *vpt, Rect *rect )
{
  if (m_stroke){
    m_stroke->GetViewportRect(t,vpt,rect);
    return;
  }
}

void SMBrushSelect::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  if (m_stroke){
    m_stroke->Proc(vpt,hit,m,lastm);
  }
}

void SMBrushSelect::StartStroke(int mode, BOOL cont, int subobj, const SMHit &hit)
{
  SM_ASSERT(m_stroke==NULL);
  SM_ASSERT(subobj==SM_SO_VERTEX || subobj==SM_SO_EDGE || subobj==SM_SO_BORDER || subobj==SM_SO_FACE || subobj==SM_SO_ELEMENT);

  if (m_SM->m_ip->SelectionFrozen())
    return;


  DbgAssert(!theHold.Holding());
  if (0 && theHold.Holding()) {
    theHold.Cancel();
  }

  theHold.Begin();

  // actually only needed when really a change was made
  if (theHold.Holding())
    theHold.Put (new ComponentFlagRestore (m_SM->m_polyobj, SMSOTypeToMNType(subobj)));


  m_stroke = m_strokes[mode%SM_SELECTS];
  m_stroke->Start(cont,subobj,hit);
}

void SMBrushSelect::EndStroke(const SMHit &hit)
{
  MCHAR* undomessage = _M("AIF Select");

  if (m_stroke){
    undomessage = m_stroke->GetUndoMessage();
    m_stroke->End(hit);

    IMeshSelect* ms = (m_SM->m_polyobj) ? GetMeshSelectInterface(m_SM->m_polyobj) : NULL;
    if (ms) ms->LocalDataChanged();

    theHold.Accept(undomessage);
  }


  m_stroke = NULL;
}

void SMBrushSelect::AbortStroke()
{
  if (m_stroke){
    m_stroke->Abort();

    theHold.Cancel();
  }
  m_stroke = NULL;
}

SMBrushSelect::~SMBrushSelect()
{
  SM_ASSERT(m_strokepacket);
  delete m_strokepacket;
}

#endif

