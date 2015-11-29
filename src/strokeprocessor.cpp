/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/
#include "strokeprocessor.h"
#include "meshconnectivity.h"
#include "meshlooping.h"

#define DELTA_DELAY   (m_deltadelay >= 0)
#define SM_STROKEPROC_CONSTRAINDFIXED 0

//////////////////////////////////////////////////////////////////////////
// StrokeProcessor
void StrokeProcessor::ConnectPoints(int self, int prev)
{
  StrokePoint *sp = &m_strokepoints[self];

  int laststartid = m_strokepoints[prev].startvid;
  for (int i = 0; i < m_numdeltas; i++){
    m_connectids[0][i] = laststartid+i;
    m_connectids[1][i] = sp->startvid+i;
  }
  if (m_closedring){
    m_connectids[1][m_numdeltas] = sp->startvid;
    m_connectids[0][m_numdeltas] = laststartid;
  }
  int cnt;
  sp->startfid = MNMesh_connectVAs(m_outmesh,m_connectids[0].Addr(0),m_connectids[1].Addr(0),m_numdeltas+m_closedring,cnt);
  for (int i = 0; i < m_lastdelta+m_closedring; i++){
    MNFace *face = m_outmesh->F(sp->startfid+i);
    face->smGroup = m_fattribs[i].smGroup;
    face->material = m_fattribs[i].material;
    face->SetFlag(SM_STRIPFACE_FLAG);
  }
}

StrokePoint& StrokeProcessor::AddPointRaw()
{
  StrokePoint sp;

  sp.flag = SM_STROKEFLAG_NONE;
  sp.startvid = m_outmesh->AppendNewVerts(m_numdeltas);
  sp.size = 1.0;
  sp.dynweld[0] = -1;
  sp.dynweld[1] = -1;

  m_lastpoint++;
  m_strokepoints.push_back(sp);

  if (m_lastpoint >= 1){
    ConnectPoints(m_lastpoint,m_lastpoint-1);
  }

  return m_strokepoints[m_lastpoint];
}

Point3 AbsPoint(const Point3& pt)
{
  return Point3(abs(pt.x),abs(pt.y),abs(pt.z));
}

void StrokeProcessor::AddPoint(const Point3 &hit, const Matrix3 &mat, float size)
{
  if (m_lastpoint == m_deltadelay){
    const Point3& pos = m_strokepoints[m_lastpoint-1].pos;
    Point3 normal = m_objtminv.VectorTransform(m_wplane->GetNormal()).Normalize();
    if (m_wplane->HasCollAlignment()){
      CollAlignNormal(pos,normal);
    }

    m_edgedeltamat = GetEdgeMat(hit,pos,normal);
    UpdateDeltas(m_edgedeltamat);
    m_deltadelay = -2;
  }

  StrokePoint& sp = AddPointRaw();
  sp.size = size;
  sp.opos = hit;
  sp.pos  = hit;

  MNVert* __restrict vert = &m_outmesh->v[sp.startvid];
  for (int i = 0; i < m_numdeltas; i++,vert++){
    vert->p = hit + (mat.VectorTransform(m_deltas[i])*size);
  }
}


void StrokeProcessor::RemoveLastPointFaces()
{
  StrokePoint *spkick= &m_strokepoints[m_lastpoint];

  for (int n = 0; n < m_lastdelta+m_closedring; n++){
    int faceid = spkick->startfid+n;
    MNFace *face = m_outmesh->F(faceid);
    face->SetFlag(MN_DEAD);
    face->ClearFlag(SM_STRIPFACE_FLAG);

    int deg = face->deg;
    for (int f = 0; f < deg; f++){
      MNEdge *edge = m_outmesh->E(face->edg[f]);
      if (edge->OtherFace(faceid) < 0 ||
        edge->OtherFace(faceid) == faceid+1)
      {
        edge->SetFlag(MN_DEAD);
      }
    }
  }
}

void StrokeProcessor::RemoveLastPoint()
{
  StrokePoint *spkick= &m_strokepoints[m_lastpoint];
  RemoveLastPointFaces();

  for (int n = 0; n < m_numdeltas; n++){
    m_outmesh->V(spkick->startvid+n)->SetFlag(MN_DEAD);
  }

  m_strokepoints.pop_back();
  m_lastpoint--;
}

void StrokeProcessor::RemoveDeadPoints()
{
  int deadcnt = 0;

  // first pass fill continously
  BOOL copylast = FALSE;
  int last = -1;
  for (int i = 0; i <= m_lastpoint; i++){
    StrokePoint *sp = &m_strokepoints[i];

    if (sp->flag & SM_STROKEFLAG_DEAD){
      copylast = TRUE;
      last = last < 0 ? i : last;
      deadcnt++;
    }
    else if (copylast){
      StrokePoint *splast = &m_strokepoints[last];
      // copy all stuff except vertex/faceids
      splast->flag  = sp->flag;
      splast->size  = sp->size;
      splast->pos    = sp->pos;
      splast->opos   = sp->opos;
      last = i;
    }
  }

  // now all "active" are a continous chunk,
  // clear the last ones
  for (int i = 0; i < deadcnt; i++){
    RemoveLastPoint();
  }

}

//////////////////////////////////////////////////////////////////////////

__forceinline Point3* StrokeProcessor::CollAlignNormal(const Point3& sp, Point3 &normal)
{
  if (m_wplane->CorrectNormalVisible(m_objtm.PointTransform(sp),normal)){
    normal = m_objtminv.VectorTransform(normal);
    return &normal;
  }
  return NULL;
}

__forceinline Point3 StrokeProcessor::CollCorrectPoint(const Point3& sp)
{
  return m_objtminv.PointTransform(m_wplane->CorrectPointVisible(m_objtm.PointTransform(sp)));
}

Matrix3 StrokeProcessor::GetEdgeMat(const Point3 &target, const Point3 &pos, const Point3 &normal)
{
  Point3 dir = (target-pos).Normalize();
  dir = VectorTransform(dir,Inverse(m_startmat));
  // snap to closest dir
  Point3 newdir = dir;
  float  angle = 0;
  for (int i = 1; i < 3; i++){
    Point3 testdir(0,0,0);
    testdir[i] = 1.0;
    float curangle = DotProd(testdir,dir);
    if ( abs(curangle) > angle) {
      newdir = testdir * fsign(curangle);
      angle = abs(curangle);
    }
  }
  for (int i = 0; i < 3; i++){
    Point3 testdir(0,0,0);
    testdir[i] = 1.0;
    testdir[(i+1)%3] = 1.0;
    testdir.Unify();

    float curangle = DotProd(testdir,dir);
    if ( abs(curangle) > angle) {
      newdir = testdir * fsign(curangle);
      angle = abs(curangle);
    }
  }
  for (int i = 0; i < 3; i++){
    Point3 testdir(0,0,0);
    testdir[i] = 1.0;
    testdir[(i+1)%3] = -1.0;
    testdir.Unify();

    float curangle = DotProd(testdir,dir);
    if ( abs(curangle) > angle) {
      newdir = testdir * fsign(curangle);
      angle = abs(curangle);
    }
  }
  for (int i = 0; i < 3; i++){
    Point3 testdir(1,1,1);
    testdir[i] *= -1;
    testdir.Unify();

    float curangle = DotProd(testdir,dir);
    if ( abs(curangle) > angle ) {
      newdir = testdir * fsign(curangle);
      angle = abs(curangle);
    }
  }

  newdir = VectorTransform(newdir,m_startmat);

  Matrix3 mat(1);
  Matrix3 smat(1);

  Point3 refdir = newdir;
  refdir -= DotProd(refdir,normal)*normal;
  refdir = refdir.Normalize();
  Point3 up = CrossProd(normal,refdir).Normalize();

  mat.SetRow(0,refdir);
  mat.SetRow(1,up);
  mat.SetRow(2,normal);

  smat.SetScale(Point3(m_objtm.GetRow(0).Length(),m_objtm.GetRow(1).Length(),m_objtm.GetRow(2).Length()));

  return smat * Inverse(mat);
}

void StrokeProcessor::DeltaMatrix(Matrix3 &mat, const Point3 &pos, const Point3 &tonext, int pt, float size)
{
  Point3  zero(0.0f,0.0f,0.0f);
  Point3  scale(size,size,size);
  Point3  next = m_obj2wp.VectorTransform(tonext).Normalize();
  Point3  normal(0.0f,0.0f,1.0f);

  if (m_wplane->HasCollAlignment() && CollAlignNormal(pos,normal)){
    normal = m_obj2wp.VectorTransform(normal);
  }

  Point3  edgedir = CrossProd(normal,next).Normalize();
  normal = CrossProd(next,edgedir).Normalize();

  if ( m_mode == STR_FACE && DotProd(m_strokepoints[1].opos-m_startmat.GetRow(3),m_startmat.GetRow(2)) < 0 )
  {
    next    = -next;
    edgedir = -edgedir;
  }

  // build workspace mat
  if (m_lastpoint != m_deltadelay)
    mat.Set(next,edgedir,normal,Point3(0.0f,0.0f,0.0f));
  else
    mat = m_obj2wp;

  if (m_rotfactor != 0.0f && m_lastpoint != m_deltadelay){
    Matrix3 rotmat = TransMatrix(-m_deltactr);
    // FIXME rotate around center of deltas
    SM_ASSERT(m_lastpoint > 0);
    float angle = m_rotfactor * DEG_TO_RAD;
    angle *= float(pt)/float(m_lastpoint);
    rotmat.RotateX(angle);
    rotmat.Translate(m_deltactr);
    mat = rotmat * mat;
  }

  // scale
  mat.Scale(scale);
  // from workplane to objspace
  mat = mat * m_wp2obj;
  // overwrite translation
  mat.SetTrans(pos);

  // apply offset
  if (m_wplane->HasCollCorrection() && m_wplane->CollGetOffset() != 0.0f){
    Point3 normal(0.0f,0.0f,0.0f);
    if (m_wplane->CorrectNormalVisible(m_objtm.PointTransform(pos),normal)){
      normal = m_objtminv.VectorTransform(normal);
      mat.SetTrans(pos+ (normal * m_wplane->CollGetOffset()));
    }
  }
}

__forceinline void StrokeProcessor::UpdateVertices(const StrokePoint &sp, const Matrix3 &mat)
{
  if (sp.flag & SM_STROKEFLAG_LOCK) return;

  int begin =  0;
  int end   =  m_numdeltas;

  MNVert* __restrict vert = m_outmesh->V(sp.startvid);
  for (int i = begin; i < end; i++,vert++){
    vert->p = (mat.PointTransform(m_deltas[i]));
  }

  if (m_mode == STR_FACE && &sp == &m_strokepoints[m_lastpoint]){
    vert = m_outmesh->V(m_capstartvid);
    for (int i = 0; i < m_numcapverts; i++,vert++){
      vert->p = (mat.PointTransform(m_capdeltas[i]));
    }
  }
}

void StrokeProcessor::SetPoint(int pt, const Point3 &hit, float size)
{
  StrokePoint& sp = m_strokepoints[pt];
  sp.opos = hit;
  sp.pos = hit;
  sp.size = size;
}

void StrokeProcessor::UpdatePoint(int pt)
{
  int next = pt+1;
  int prev = pt-1;
  if (pt == m_lastpoint){
    next = pt;
  }
  if (pt == 0){
    prev = 0;
  }

  const StrokePoint&  spprev = m_strokepoints[prev];
  StrokePoint&            sp = m_strokepoints[pt];
  const StrokePoint&  spnext = m_strokepoints[next];

  // half of next and prev
  DeltaMatrix(sp.deltamat,sp.pos,spnext.pos-spprev.pos,pt,sp.size);
  UpdateVertices(sp,sp.deltamat);
}

static bool AdvanceWpair(WeldPair& wpair, MNMesh* refmesh)
{
  if (wpair.nextedge < 0) return false;
  wpair.ovid     = refmesh->E(wpair.nextedge)->OtherVert(wpair.ovid);
  wpair.nextedge = MNMesh_nextEdgeLoopMax(refmesh,wpair.nextedge,wpair.ovid);

  return true;
}

BOOL StrokeProcessor::DynamicWeld(int from, int to, WeldMode mode)
{
  if (!m_dynweld || !m_dynweldfactor) return FALSE;

  from = max(from,1);
  m_lastdynamic[0] = 0;
  m_lastdynamic[1] = 0;
  // reset to last wpairs
  for (int i = 1; i <= to; i++){
    for (int v = 0; v < 2; v++){
      if (i < from && m_strokepoints[i].dynweld[v] >= 0){
        m_lastdynamic[v] = m_strokepoints[i].dynweld[v];
      }
      else{
        m_strokepoints[i].dynweld[v] = -1;
      }
    }
  }
  for (int v = 0; v < 2; v++){
    int deletecnt = m_dynwpairs[v].size() - (m_lastdynamic[v]+1);
    for (int i = 0; i < deletecnt; i++){
      m_dynwpairs[v].pop_back();
    }
  }

  // rebuild wpairs from now on
  BOOL hadweld = FALSE;
  for (int i = from; i <= to; i++){
    StrokePoint& sp = m_strokepoints[i];
    float weldDistance  = (sp.pos - m_strokepoints[i-1].pos).Length() * m_dynweldfactor;
    int weldcnt = 0;
    Point3 welddelta = Point3(0,0,0);
    int begin = 0;
    int end   = m_lastdelta;

    // test outer vertices against edges starting from lastdynamic
    for (int v = 0; v < 2; v++){
      WeldPair wp = m_dynwpairs[v][m_lastdynamic[v]];
      wp.avid = -1;

      int vtx = sp.startvid + v*m_lastdelta;
      Point3& vtxpt = m_outmesh->P(vtx);

      float lastdistance = 10000000.0;
      while (AdvanceWpair(wp,m_refmesh)){
        Point3& refpt = m_refmesh->P(wp.ovid);
        Point3 delta = (refpt - vtxpt);
        float distance = delta.Length();
        if ( distance < weldDistance ){
          welddelta += delta;
          weldcnt++;
          hadweld = TRUE;

          {
            wp.avid = vtx;
            vtxpt = refpt;
            m_lastdynamic[v]++;
            m_dynwpairs[v].push_back(wp);
            sp.dynweld[v] = m_lastdynamic[v];

            if (v==0) begin = 1;
            if (v==1) end   = m_lastdelta-1;
          }
          break;
        }
        else if (distance > lastdistance){
          // assume we only can get "closer" 
          break;
        }

        lastdistance = distance;
      }
    }

    welddelta /= float(weldcnt);
    if (weldcnt && mode == WELD_SNAPALLVERTS){
      for (int i = begin; i <= end; i++){
        m_outmesh->P(sp.startvid + i) += welddelta;
      }
    }
  }


  return hadweld;
}


void StrokeProcessor::DynamicWeldFinalize()
{
  if (m_dynweld){
    for (int v=0; v < 2; v++){
      for (int i=1; i <= m_lastdynamic[v]; i++){
        m_wpairs.push_back(m_dynwpairs[v][i]);
        m_lastweld++;
      }
      // assure that welds are not pushed twice
      m_lastdynamic[v] = 0;
    }
    
  }
}


BOOL StrokeProcessor::OverlapCheck()
{
  int quads = m_numdeltas+m_closedring-1;

  if (m_lastpoint >= 2){

    int startvid0 = m_strokepoints[m_lastpoint-0].startvid;
    int startvid1 = m_strokepoints[m_lastpoint-1].startvid;
    int startvid2 = m_strokepoints[m_lastpoint-2].startvid;


    // iterate all quads and check whether a single quad
    // has triangles with mis-matching facenormals

    for (int i = 0; i < quads; i++){
      Point3 quadref[4];
      quadref[0] = m_outmesh->P(startvid2 + (i));
      quadref[1] = m_outmesh->P(startvid2 + ((i+1)%m_numdeltas));
      quadref[2] = m_outmesh->P(startvid1 + ((i+1)%m_numdeltas));
      quadref[3] = m_outmesh->P(startvid1 + (i));

      Point3 quadpts[4];
      quadpts[0] = m_outmesh->P(startvid1 + (i));
      quadpts[1] = m_outmesh->P(startvid1 + ((i+1)%m_numdeltas));
      quadpts[2] = m_outmesh->P(startvid0 + ((i+1)%m_numdeltas));
      quadpts[3] = m_outmesh->P(startvid0 + (i));
      // 1 2
      // 0 3

      Point3 normref = CrossProd(quadref[3]-quadref[0],quadref[1]-quadref[0]);
      Point3 normcheck;
      normcheck = CrossProd(quadpts[3]-quadpts[0],quadpts[1]-quadpts[0]);
      if (DotProd(normref,normcheck) < 0.0f)
        return TRUE;
      normcheck = CrossProd(quadpts[1]-quadpts[2],quadpts[3]-quadpts[2]);
      if (DotProd(normref,normcheck) < 0.0f)
        return TRUE;
      normcheck = CrossProd(quadpts[2]-quadpts[0],quadpts[1]-quadpts[0]);
      if (DotProd(normref,normcheck) < 0.0f)
        return TRUE;
      normcheck = CrossProd(quadpts[2]-quadpts[3],quadpts[1]-quadpts[3]);
      if (DotProd(normref,normcheck) < 0.0f)
        return TRUE;

    }
  }
  return FALSE;
}

int StrokeProcessor::AddInterPoints(Point3 &hit, float size, Point3 &tohit,float dist, float stride, int maxpts)
{
  Matrix3 mat(1);

  // find out how many points
  // linear interpolate along
  int cnt = max(0,int(floor(dist/stride)));
  Point3  lasthit = m_strokepoints[m_lastpoint-1].opos;
  float lastsize = m_strokepoints[m_lastpoint-1].size;

  Point3  tohitn = tohit.Normalize();
  float fracc = 1.0f/float(cnt+1);
  float runner = fracc;

  // first old last gets correct distnace
  int oldlast = m_lastpoint;
  Point3 newpt = lasthit + (tohitn*stride*float(0+1));
  if (m_wplane->HasCollCorrection()){
    newpt = CollCorrectPoint(newpt);
  }
  SetPoint(m_lastpoint,newpt,LERP(lastsize,size,fracc));
#if SM_STROKEPROC_CONSTRAINDFIXED
  m_strokepoints[m_lastpoint].flag |= m_wplane->IsConstrained() ? SM_STROKEFLAG_NORELAX : 0;
#endif


  // create inbetweens
  runner += fracc;
  cnt = min(cnt,maxpts);
  for (int i = 0; i < cnt-1; i++){
    newpt = lasthit + (tohitn*stride*float(i+2));
    if (m_wplane->HasCollCorrection()){
      newpt = CollCorrectPoint(newpt);
    }
    AddPoint(newpt,mat,LERP(lastsize,size,runner));
    runner += fracc;
#if SM_STROKEPROC_CONSTRAINDFIXED
    //m_strokepoints[m_lastpoint].flag |= m_wplane->IsConstrained() ? SM_STROKEFLAG_NORELAX : 0;
#endif
  }

  // new end is current
  AddPoint(hit,mat,size);

  return m_lastpoint-cnt;
}


void StrokeProcessor::RelaxPoints()
{
  // fill in relax points
  StrokePoint*  spfirst = &m_strokepoints[1];
  StrokePoint*  splast = &m_strokepoints[m_lastpoint];

  StrokePoint*__restrict sp = spfirst;
  while (sp < splast){
    sp->pos = sp->opos;
    sp++;
  }

  for (int r = 0; r < m_relaxruns; r++){
    sp = spfirst;
    if (m_wplane->HasCollCorrection()){
      while (sp < splast){
#if SM_STROKEPROC_CONSTRAINDFIXED
        if (!(sp->flag & SM_STROKEFLAG_NORELAX))
#endif
        {
          Point3 spavg = ((sp-1)->pos + (sp+1)->pos)*0.5f;
          Point3 pos = LERP(sp->pos,spavg,m_relaxfactor);
          sp->deltamat.SetTrans(CollCorrectPoint(pos));
        }
        sp++;
      }
    }
    else {
      while (sp < splast){
#if SM_STROKEPROC_CONSTRAINDFIXED
        if (!(sp->flag & SM_STROKEFLAG_NORELAX))
#endif
        {
          Point3 spavg = ((sp-1)->pos + (sp+1)->pos)*0.5f;
          Point3 pos = LERP(sp->pos,spavg,m_relaxfactor);
          //sp->pos = pos;
          sp->deltamat.SetTrans(pos);
        }
        sp++;
      }
    }
    sp = spfirst;
    while (sp < splast){
#if SM_STROKEPROC_CONSTRAINDFIXED
      if (!(sp->flag & SM_STROKEFLAG_NORELAX))
#endif
      {
        sp->pos = sp->deltamat.GetTrans();
      }
      sp++;
    }
  }
  

}

__forceinline float StrokeProcessor::GetStride(float brushsize)
{
  return m_explicitstride ? 
        brushsize : 
      ( m_mode == STR_FREE ? 
          m_stridemulti * brushsize * 2.0f :
          m_stridemulti * m_deltasize);
}

__forceinline float StrokeProcessor::GetSize(float brushsize)
{
  return m_mode == STR_FREE ? brushsize : 1.0f;
}

void StrokeProcessor::CheckHit(Point3 &hit, float brushsize,ViewExp *vpt)
{

  if (m_mode != STR_EDGE &&
      m_mode != STR_FACE &&
      m_mode != STR_FREE)
  {
        return;
  }

  DbgAssert(m_lastpoint > 0);

  Point3  tohit = hit-m_strokepoints[m_lastpoint-1].opos;
  float   dist  = tohit.Length();

  BOOL isconstrained  = m_wplane->IsConstrained();
  BOOL tglconstrained = isconstrained != m_lastconstrained;

  if (dist < 0.005)
    return;
  
  m_lastconstrained = isconstrained;

  if (m_lastpoint >= 2){
    // check if we go back and remove last point accordingly
    Point3 oconstr = m_strokepoints[m_lastpoint-2].opos;
    Point3 lastdir = m_strokepoints[m_lastpoint-1].opos - m_strokepoints[m_lastpoint-2].opos;
    if ( DotProd(tohit,lastdir) < 0 && (oconstr-hit).LengthSquared() < (oconstr-m_strokepoints[m_lastpoint-1].opos).LengthSquared() )
    {
      RemoveLastPoint();
      m_outmesh->CollapseDeadStructs();
      m_outmesh->FillInMesh();
      tohit = hit-m_strokepoints[m_lastpoint-1].opos;
      dist  = tohit.Length();

      if (m_lastpoint == 1 && m_mode == STR_EDGE){
        m_deltadelay = 1;
        m_deltas = m_deltasorig;
        m_edgedeltamat.IdentityMatrix();
      }
    }
  }

  float stride = GetStride(brushsize);
  float size = GetSize(brushsize);

  int dirtystart = m_lastpoint;

  if (dist >= stride){
    dirtystart = AddInterPoints(hit,size,tohit,dist,stride);
  }
  if (m_relaxruns > 0 && m_relaxfactor > 0.0f){
    RelaxPoints();
    dirtystart = 1;
  }
  if (m_rotfactor != 0.0f || m_rotfactorold != m_rotfactor){
    dirtystart = 1;
    m_rotfactorold = m_rotfactor;
  }

  // modify current points
  // set current point
  SetPoint(m_lastpoint,hit,size);

  // half/full rotate last points
  dirtystart = max(0,min(dirtystart,m_lastpoint-1));
  if (m_mode == STR_FREE && dirtystart == 1){
    dirtystart = 0;
  }
  UpdateAll(dirtystart,m_lastpoint);
  DynamicWeld(dirtystart,m_lastpoint,m_dynweldmode);


  if (m_wplane->HasCollCorrection()){
    m_wplane->SetPoint(m_objtm.PointTransform(m_strokepoints[m_lastpoint-1].pos));
  }

  m_outmesh->InvalidateGeomCache();
}

void StrokeProcessor::UpdateDeltas(const Matrix3 &tm)
{
  for (int i = 0; i < m_numdeltas; i++){
    m_deltas[i] = tm.PointTransform(m_deltas[i]);
  }
  m_deltactr = tm.PointTransform(m_deltactr);

  for (int i = 0; i < m_numcapverts; i++){
    m_capdeltas[i] = tm.PointTransform(m_capdeltas[i]);
  }
}

void StrokeProcessor::GetFaceAttribs(const Tab<int> &edges, int startface, Tab<FaceAttrib> &fattribs, DWORD &sgs)
{
  fattribs.SetCount(edges.Count());
  for (int i = 0; i < edges.Count(); i++) {
    MNEdge  *curedge = m_refmesh->E(edges[i]);
    int faceid = (startface >= 0 && curedge->f2 >= 0 && m_faces[curedge->f1]) ? curedge->f2 : curedge->f1;
    MNFace  *face = m_refmesh->F(faceid);

    fattribs[i].face = faceid;
    fattribs[i].smGroup = face->smGroup;
    fattribs[i].material = face->material;

    sgs |= face->smGroup;
  }
}

static __forceinline DWORD findLastBit(DWORD bits)
{
  for (int i = (sizeof(DWORD)*8)-1; i >= 0 ; i--){
    if (bits & (1<<i))
      return (1<<i);
  }
  return 0;
}

static __forceinline int countBits(DWORD bits)
{
  int cnt = 0;
  for (int i = (sizeof(DWORD)*8)-1; i >= 0 ; i--){
    if (bits & (1<<i))
      cnt++;
  }
  return cnt;
}

void StrokeProcessor::InitDeltaSize()
{
  int numfaces = m_numdeltas-1+m_closedring;
  float minsize = FLT_MAX;
  float maxsize = 0;
  float sumsize = 0;

  m_deltactr.Set(0,0,0);
  for (int i = 0; i < numfaces; i++){
    float length = (m_deltas[i]-m_deltas[i+1]).Length();
    minsize = min(length,minsize);
    maxsize = max(length,maxsize);
    sumsize += length;
    m_deltactr += m_deltas[i];
  }
  m_deltactr += m_deltas[numfaces];
  m_deltactr /= float(numfaces+1);

  switch (m_deltastridemode)
  {
  case DELTASIZE_AVG:
    m_deltasize = sumsize/float(numfaces);
    break;
  case DELTASIZE_MAX:
    m_deltasize = maxsize;
    break;
  case DELTASIZE_MIN:
    m_deltasize = minsize;
    break;
  }
}

void StrokeProcessor::InitFromEdgeOrFace(Tab<int> &edgeverts, const Point3 &hit, int startedge, int startface, float size, const Matrix3 &wpmat, const Matrix3 &wpmatinv)
{

  // get edges and vertices
  GetStartEdges(startedge,startface,hit,size,m_startedges,edgeverts);
  
  MNEdge *edge = m_refmesh->E(m_startedges[0]);
  if (startface >= 0 && !(
      (MNMesh_isFaceSelected(m_refmesh,edge->f1) && edgeverts[0] == edge->v1) ||
      (edge->f2 >= 0 && MNMesh_isFaceSelected(m_refmesh,edge->f2) && edgeverts[0] == edge->v2)))
  {
    // reverse order
    Tab<int> old = m_startedges;
    for (int i = 0; i < old.Count(); i++)  {
      m_startedges[i] = old[m_startedges.Count()-1-i];
    }
    old = edgeverts;
    for (int i = 0; i < old.Count(); i++)  {
      edgeverts[i] = old[old.Count()-1-i];
    }
  }

  edge = m_refmesh->E(startedge);

  // flag edges to prevent self-overlap at end connection
  m_openendedges.SetAll();
  for (int i = 0; i < m_startedges.Count(); i++)  {
    m_openendedges.Clear(m_startedges[i]);
  }
  // setup face attributes
  GetFaceAttribs(m_startedges,startface,m_fattribs,m_invalidsgs);

  // find if closed edges
  m_numdeltas = edgeverts.Count();
  if (edgeverts[0] == edgeverts[m_numdeltas-1]){
    m_numdeltas--;
    m_closedring = TRUE;
  }
  else{
    m_closedring = FALSE;
  }

  // init counts
  m_lastdelta = m_numdeltas-1;
  SM_ASSERT(m_numdeltas > 1);

  m_deltas.SetCount(m_numdeltas+m_closedring);
  m_connectids[0].SetCount(m_numdeltas+m_closedring);
  m_connectids[1].SetCount(m_numdeltas+m_closedring);

  // get starting infos
  m_startfracc = (hit-m_refmesh->P(edge->v1)).Length()/(m_refmesh->P(edge->v2)-m_refmesh->P(edge->v1)).Length();

  for (int i = 0; i < m_numdeltas+m_closedring; i++){
    Point3 pt = m_refmesh->P(edgeverts[i]);
    pt -= hit;
    m_deltas[i] = pt;
  }

  InitDeltaSize();

  // add first points
  AddPoint(hit,m_wp2obj,1.0f);
  // fix first point
  StrokePoint *sp = &m_strokepoints[0];
  sp->flag |= SM_STROKEFLAG_LOCK;
  for (int i = 0; i < m_numdeltas; i++){
    m_outmesh->V(sp->startvid+i)->p = m_refmesh->P(edgeverts[i]);
  }
  AddPoint(hit,m_wp2obj,1.0f);
  m_lastpoint=1;


  // weldpairs for all first
  int startid = m_strokepoints[0].startvid;
  for (int i = 0; i < m_numdeltas; i++){
    WeldPair wpair;
    wpair.avid = startid+i;
    wpair.ovid = edgeverts[i];

    if (startface >= 0){
      Tab<int> &vfaces = m_refmesh->vfac[wpair.ovid];
      int cnt = vfaces.Count();
      int selcnt = 0;
      // only add weld if vertex has non-selected faces
      // because hat vertex will be deleted otherwise
      // once the old cap is deleted
      for (int n = 0; n < cnt; n++){
        selcnt += m_refmesh->f[vfaces[n]].GetFlag(MN_SEL) ? 1 : 0;
      }
      if (selcnt == cnt){
        continue;
      }
    }
    m_wpairs.push_back(wpair);
  }
  m_lastweld = m_lastdelta;

  m_startmat = MNMesh_getFaceMatrix(m_refmesh,edge->f1,startedge,hit);
}

void StrokeProcessor::StartFromEdge(Point3 &hit, int startedge, float size, const Matrix3 &wpmat, const Matrix3 &wpmatinv)
{
  // first setup extrusion deltas and welds
  Tab<int> edgeverts;
  InitFromEdgeOrFace(edgeverts,hit,startedge,-1,size,wpmat,wpmatinv);

  // now check for dynamic weld or rail mode
  // find if last/first is corner (enable weld)
  // make sure last weld is the corner weldpair

  m_rail = 0;
  if (!m_closedring){
    MNEdge *edge = m_refmesh->E(startedge);
    BOOL  startselected = edge->GetFlag(MN_SEL);
    int   corner = -1;
    for (int v = 0; v < 2; v++){
      m_dynwpairs[v].resize(1);
      m_dynwpairs[v][0].ovid = -1;
      m_dynwpairs[v][0].avid = -1;
      m_dynwpairs[v][0].nextedge = -1;
    }

    for (int v = 0; v < 2; v++){
      WeldPair& wpair = m_dynwpairs[v][0];
      wpair = m_wpairs[v*m_lastdelta];
      wpair.nextedge = -1;

      // find nextedge
      Tab<int> &edges = m_refmesh->vedg[wpair.ovid];
      int edgecnt = edges.Count();
      for (int i = 0; i < edgecnt; i++){
        int edgeid = edges[i];
        if(edgeid == startedge)
          continue;

        MNEdge *edge = m_refmesh->E(edgeid);
        if (edge->f2 < 0 && m_openendedges[edgeid])
        {
          wpair.nextedge = edgeid;
          break;
        }
      }

      if (wpair.nextedge >= 0){
        m_dynweld = TRUE;
        if (edgecnt >= 2 && m_railmode != RAIL_NONE){
          m_rail |= 1<<v;
        }
      }
    }
  }
  m_edgedeltamat.IdentityMatrix();

  if ( m_rail ){
    m_rail = 0;
    // anchor based on start face/edge

    // compute current 
  }
  else{
    m_deltadelay = 1;
    if (m_wplane->HasCollCorrection()){
      m_strokepoints[0].pos = m_strokepoints[0].opos = CollCorrectPoint(m_strokepoints[0].opos);
    }
  }
}

void StrokeProcessor::StartFromFace(Point3 &hit, int startedge, int startface, float size, const Matrix3 &wpmat, const Matrix3 &wpmatinv, const Matrix3* clustermatrix)
{
  // first setup extrusion deltas and welds
  Tab<int> edgeverts;
  InitFromEdgeOrFace(edgeverts,hit,startedge,startface,size,wpmat,wpmatinv);

  // find capfaces
  Tab<int> refverts;
  BitArrayToTab getverts(m_refmesh,&refverts);
  SM_BITARRAY_ENUMSET(m_faceverts,getverts,BitArrayToTab);

  int vcnt = refverts.Count();

  std::map<int,int> reftonew;

  // preinit
  for (int i = 0; i < vcnt; i++){
    reftonew[refverts[i]] = -1;
  }

  int vid = m_outmesh->AppendNewVerts(vcnt);
  MNVert  *vert = m_outmesh->V(vid);

  m_numcapverts = vcnt;
  m_capstartvid = vid;

  // setup edgeverts first (for easier endwelding)
  for (int i = 0; i < m_numdeltas; i++,vert++,vid++){
    reftonew[edgeverts[i]] = vid;
    vert->p = m_refmesh->P(edgeverts[i]);
  }
  // then fill in all unset
  for (int i = 0; i < vcnt; i++){
    int &curvid = reftonew[refverts[i]];
    if (curvid >= 0) continue;

    curvid = vid;
    vert->p = m_refmesh->P(refverts[i]);
    vert++;
    vid++;
  }

  // build deltaverts
  m_capdeltas.SetCount(vcnt);
  for (int i = 0; i < vcnt; i++){
    Point3 pt = m_outmesh->P(m_capstartvid + i);
    pt -= hit;
    m_capdeltas[i] = pt;
  }
  m_capdeltasorig = m_capdeltas;
  m_capstartfid = -1;
  m_numcapfaces = 0;

  std::vector<int> faceverts;

  for (int i = 0; i < vcnt; i++){
    // go thru all faces for the vertices and check whether they were selected
    // if yes, then copy them, and remove from selection
    Tab<int> &confaces = m_refmesh->vfac[refverts[i]];
    int facecnt = confaces.Count();
    for (int f = 0; f < facecnt; f++){
      int faceid = confaces[f];
      if (!m_faces[faceid]) continue;
      MNFace *face = m_refmesh->F(faceid);
      int deg = face->deg;

      // convert vertex ids
      faceverts.clear();
      for (int n = 0; n < deg; n++){
        faceverts.push_back(reftonew[face->vtx[n]]);
      }

      // create face
      int curface = m_outmesh->CreateFace(face->deg,&faceverts[0]);
      if (curface >= 0){
        m_capstartfid = m_capstartfid < 0 ? curface : m_capstartfid;
        m_invalidsgs |= face->smGroup;
        m_numcapfaces++;
      }

      // remove
      m_faces.Set(faceid,FALSE);
      // mark for deletion
      m_delfaces.push_back(faceid);
    }
  }

  // setup matrix
  {
    SM_ASSERT(clustermatrix);
    //m_edgedeltamat = GetEdgeMat(clustermatrix->GetRow(2));

    Matrix3 smat(1);
    smat.SetScale(Point3(m_objtm.GetRow(0).Length(),m_objtm.GetRow(1).Length(),m_objtm.GetRow(2).Length()));


    Point3 normal = m_objtminv.VectorTransform(m_wplane->GetNormal()).Normalize();
    float dotx = DotProd(normal,clustermatrix->GetRow(0));
    float doty = DotProd(normal,clustermatrix->GetRow(1));

    m_edgedeltamat.IdentityMatrix();
    m_edgedeltamat.SetRow(0,clustermatrix->GetRow(2));
    if (abs(dotx) > abs(doty)){
      normal = clustermatrix->GetRow(0)*fsign(dotx);
    }
    else{
      normal = clustermatrix->GetRow(1)*fsign(doty);
    }
    m_edgedeltamat.SetRow(1,CrossProd(normal,clustermatrix->GetRow(2)).Normalize());
    m_edgedeltamat.SetRow(2,normal);

    m_startmat.SetRow(0,m_edgedeltamat.GetRow(1));
    m_startmat.SetRow(1,m_edgedeltamat.GetRow(2));
    m_startmat.SetRow(2,m_edgedeltamat.GetRow(0));
    m_startmat.SetRow(3,hit);

    m_edgedeltamat = smat * Inverse(m_edgedeltamat);

    Matrix3 startinv = Inverse(m_startmat);

    Box3 bbox;
    bbox.Init();
    for (int i = 0; i < m_numdeltas; i++){
      bbox += startinv.VectorTransform(m_deltas[i]);
    }

    m_startrelative = (-bbox.Min()) / (bbox.Width());
  }
}


void StrokeProcessor::StartFromFree( Point3 & hit, float size )
{
  m_deltas    = m_freedeltas;
  m_numdeltas = m_deltas.Count();
  m_connectids[0].SetCount(m_numdeltas);
  m_connectids[1].SetCount(m_numdeltas);

  int numfaces = m_numdeltas-1;
  m_fattribs.SetCount(numfaces);
  for (int i = 0; i < numfaces; i++){
    m_fattribs[i].material = 0;
    m_fattribs[i].smGroup = 0;
  }


  // check for closed ring
  m_closedring = FALSE;
  if (m_deltas[0].Equals(m_deltas[m_numdeltas-1])){
    m_closedring = TRUE;
    m_numdeltas--;
  }
  m_lastdelta = m_numdeltas-1;

  InitDeltaSize();

  // rotate 
  if (m_rotfree != 0){
    Matrix3 rotmat = TransMatrix(-m_deltactr);
    float angle = m_rotfree * DEG_TO_RAD;
    rotmat.RotateX(angle);
    rotmat.Translate(m_deltactr);
    UpdateDeltas(rotmat);
  }

  // create two hits
  AddPoint(hit,Matrix3(1),size);
  AddPoint(hit,Matrix3(1),size);
  m_lastpoint=1;
  m_edgedeltamat.IdentityMatrix();
}

void StrokeProcessor::Start(Point3 &hit, int startedge,int startfaceid, float size, const Matrix3* clustermatrix)
{
  m_startedge = startedge;
  m_startface = startfaceid;
  m_rotfactorold = m_rotfactor;
  m_lastconstrained = m_wplane->IsConstrained();
  m_ended = false;

  if (startfaceid >= 0){
    m_mode = STR_FACE;
  }
  else if (startedge >= 0){
    m_mode = STR_EDGE;
  }
  else{
    m_mode = STR_FREE;
  }

  m_closedring = FALSE;
  m_dynweld = FALSE;
  m_deltadelay = -2;
  m_invalidsgs = 0;
  m_lastpoint = -1;

  Matrix3 wpmat = m_wplane->GetMatrix();
  Matrix3 wpmatinv =  m_wplane->GetMatrixInv();

  m_wp2obj = wpmat * m_objtminv;
  m_obj2wp = m_objtm * wpmatinv;

  // simple case, just use default deltas
  if (m_mode == STR_FREE){
    StartFromFree(hit, size);
  }
  else if (m_mode == STR_FACE){
    StartFromFace(hit,startedge,startfaceid,size,wpmat,wpmatinv,clustermatrix);
  }
  else if (m_mode == STR_EDGE){
    StartFromEdge(hit,startedge,size,wpmat,wpmatinv);
  }

  m_deltasorig = m_deltas;
  m_deltactrorig = m_deltactr;

  if (!DELTA_DELAY){
    UpdateDeltas(m_edgedeltamat);
  }
  m_startstride = GetStride(size);
}

Point3 StrokeProcessor::ReprojectPoint(const Point3 &pt, float offset)
{
  Point3 outpt = pt;
  Point3 viewdir;
  Point3 viewpos;
  BOOL  viewortho = m_wplane->GetViewParams(viewdir,viewpos);

  viewdir = m_objtminv.VectorTransform(viewdir);
  viewpos = m_objtminv.PointTransform(viewpos);
  Point3 wnormal   = m_objtminv.VectorTransform(m_wplane->GetNormal()).Normalize();

  float ptdist = DotProd(wnormal, pt) + offset;

  if (viewortho)
  { // orthographic
    Ray ray;
    ray.dir = -viewdir;
    ray.p = pt-(ray.dir*10.0f);
    if (!PlaneIntersect(ray,wnormal,ptdist,outpt)){
      ray.dir = viewdir;
      ray.p = pt-(ray.dir*10.0f);
      PlaneIntersect(ray,wnormal,ptdist,outpt);
    }
  }
  else
  { // perspective
    Ray ray;
    ray.dir = (pt-viewpos).Normalize();
    ray.p = viewpos;
    PlaneIntersect(ray,wnormal,ptdist,outpt);
  }

  return outpt;
}

void StrokeProcessor::EndLerpStrokepoints(const Point3 &endpt, const Matrix3& endmat, bool reproject)
{
  // lerp strokepoints distances to end
  if (m_wplane->HasCollCorrection())
    return;


  Point3 startpos;
  Point3 endpos;
  int startindex = 0;

  // reproject points along viewdirection
  startpos = m_strokepoints[0].pos;
  endpos = endpt;

  if (reproject){
    Point3 viewdir;
    Point3 viewpos;
    BOOL   viewortho = m_wplane->GetViewParams(viewdir,viewpos);
    Point3 wnormal   = m_objtminv.VectorTransform(m_wplane->GetNormal()).Normalize();

    viewdir = m_objtminv.VectorTransform(viewdir);
    viewpos = m_objtminv.PointTransform(viewpos);

    if (viewortho)
    { // orthographic
      float fracc = 1.0f/float(m_lastpoint-startindex);
      float runner = fracc;

      for (int i = startindex+1; i < m_lastpoint; i++){
        StrokePoint *sp = &m_strokepoints[i];
        Ray ray;
        ray.dir = -viewdir;
        ray.p = sp->pos-(ray.dir*10.0f);
        Point3 planepos = CATMULLROM(startpos,startpos,endpos,endpos,runner);
        if (!PlaneIntersect(ray,wnormal,planepos,sp->pos)){
          ray.dir = viewdir;
          ray.p = sp->pos-(ray.dir*10.0f);
          if (!PlaneIntersect(ray,wnormal,planepos,sp->pos)){
            sp->opos = sp->pos = planepos;
          }
        }

        runner += fracc;
      }
    }
    else
    { // perspective
      float fracc = 1.0f/float(m_lastpoint-startindex);
      float runner = fracc;

      for (int i = startindex+1; i < m_lastpoint; i++){
        StrokePoint *sp = &m_strokepoints[i];
        Ray ray;
        ray.dir = (sp->pos-viewpos).Normalize();
        ray.p = viewpos;
        Point3 planepos = CATMULLROM(startpos,startpos,endpos,endpos,runner);
        if (!PlaneIntersect(ray,wnormal,planepos,sp->pos)){
          sp->pos = planepos;
        }

        runner += fracc;
      }
    }
  }

  {
    Point3  smoothstartdir = (m_strokepoints[startindex+1].pos - m_strokepoints[startindex].pos).Normalize();
    Point3  smoothenddir   = (m_strokepoints[m_lastpoint].pos  - m_strokepoints[m_lastpoint-1].pos).Normalize();

    StrokePointArray  oldpts;
    oldpts.reserve(m_lastpoint+1);
    oldpts.push_back(m_strokepoints[0]);

    // remove strokepoints too close to each other, or with too big change
    // in direction
    Point3  lastdir = (m_strokepoints[1].pos - m_strokepoints[0].pos).Normalize();
    Point3  lastpos = m_strokepoints[0].pos;
    for (int i = 1; i < m_lastpoint; i++){
      StrokePoint &sp = m_strokepoints[i];

      Point3  dir = (sp.pos-lastpos);
      float dist = dir.LengthUnify();

      if (dist >= m_deltasize*m_stridemulti*0.5f &&
        DotProd(lastdir,dir) >= 0.2f)
      {
        lastdir = dir;
        lastpos = sp.pos;
        oldpts.push_back(sp);
      }

      sp.flag |= SM_STROKEFLAG_DEAD;
    }
    RemoveDeadPoints();
    oldpts.push_back(m_strokepoints[m_lastpoint]);
    int oldcnt = (int)oldpts.size();

    m_outmesh->CollapseDeadStructs();
    m_outmesh->FillInMesh();

    // for each point compute relative distance along path
    float pathdistance = 0.0f;
    oldpts[0].onpath = 0.0f;
    for (int i = 1; i < oldcnt; i++){
      StrokePoint &sp = oldpts[i];
      pathdistance += Distance(oldpts[i-1].pos, sp.pos);
      sp.onpath = pathdistance;
    }

    // oldpts = first, ... welded, ... inbetween, end
    // m_strokepoints = first, ... welded, end


    // setup interpolation
    Point3  smoothprev;
    Point3  smoothfrom = m_strokepoints[startindex].pos;
    Point3  smoothto   = m_strokepoints[m_lastpoint].pos;
    Point3  smoothnext;

    // facematrix
    // x = edge (ccw)
    // y = inside
    // z = face normal
    // deltamatrix
    // x = dir
    // y = edge
    // z = normal
    Matrix3 smooth(1);

    if (DotProd(m_startmat.GetRow(1),smoothstartdir) > 0 && abs(DotProd(m_startmat.GetRow(2),smoothstartdir)) < RAD_OF_45DEG ){
      // connect in face plane
      smooth.SetRow(0, m_startmat.GetRow(1));
      smooth.SetRow(1,-m_startmat.GetRow(0));
      smooth.SetRow(2, m_startmat.GetRow(2));
    }
    else if (DotProd(m_startmat.GetRow(2),smoothstartdir) < 0){
      // connect top
      smooth.SetRow(0,-m_startmat.GetRow(2));
      smooth.SetRow(1,-m_startmat.GetRow(0));
      smooth.SetRow(2, m_startmat.GetRow(1));
    }
    else{
      // connect bottom
      smooth.SetRow(0, m_startmat.GetRow(2));
      smooth.SetRow(1,-m_startmat.GetRow(0));
      smooth.SetRow(2,-m_startmat.GetRow(1));
    }

    smooth.PreRotateY(-m_interpolator.tangentweight.z * DEG_TO_RAD);

    smoothprev = smooth.VectorTransform(Point3(1,0,0)).Normalize();

    if (DotProd(-endmat.GetRow(1),smoothenddir) > 0 && abs(DotProd(endmat.GetRow(2),smoothenddir)) < RAD_OF_45DEG ){
      // connect in face plane
      smooth.SetRow(0,-endmat.GetRow(1));
      smooth.SetRow(1, endmat.GetRow(0));
      smooth.SetRow(2, endmat.GetRow(2));
    }
    else if (DotProd(endmat.GetRow(2),smoothenddir) > 0){
      // connect bottom
      smooth.SetRow(0, endmat.GetRow(2));
      smooth.SetRow(1, endmat.GetRow(0));
      smooth.SetRow(2, endmat.GetRow(1));
    }
    else{
      // connect top
      smooth.SetRow(0,-endmat.GetRow(2));
      smooth.SetRow(1, endmat.GetRow(0));
      smooth.SetRow(2,-endmat.GetRow(1));
    }

    smooth.PreRotateY(m_interpolator.tangentweight.w * DEG_TO_RAD);

    smoothnext = smooth.VectorTransform(Point3(1,0,0)).Normalize();

    if (m_interpolator.type == SM_INTERPOLATOR_BEZIER){
      SetupBezier(smoothfrom,smoothto,smoothprev,smoothnext,
        Point2(m_interpolator.tangentweight.x,m_interpolator.tangentweight.y));
    }
    else if (m_interpolator.type == SM_INTERPOLATOR_CATMULLROM){
      SetupCatmullRom(smoothfrom,smoothto,smoothprev,smoothnext,
        Point2(m_interpolator.tangentweight.x,m_interpolator.tangentweight.y));
    }

    // now "add points" based on subdiv lerp
    // always addpoint, and
    float stride = GetStride(m_strokepoints[0].size);
    BOOL couldadd = TRUE;
    int index = startindex;

    lastpos = m_strokepoints[m_lastpoint].pos;
    while (index+1 < oldcnt && m_lastpoint < 10000){
      // we can ignore the matrix, as the next step is gonna
      // orient the vertices anyway
      Matrix3 mat(1);
      const Point3& curveprev   = oldpts[max(index-1,0)].pos;
      const Point3& curvecur    = oldpts[index].pos;
      const Point3& curvenext   = oldpts[index+1].pos;
      const Point3& curvenext2  = oldpts[min(index+2,oldcnt-1)].pos;
      const float   curvedist     = (curvecur-curvenext).Length();

      const int steps = ((int)ceil(curvedist/stride))*100;
      const float fracc = 1.0f/float(steps);

      float runner = 0.0f;
      for (int i = 0; i < steps && m_lastpoint < 10000; i++){
        StrokePoint& spprev = m_strokepoints[m_lastpoint-1];
        Point3 curvepos = CATMULLROM(curveprev,curvecur,curvenext,curvenext2,runner);
        float smoothfracc = (oldpts[index].onpath + runner * curvedist) / pathdistance;
        Point3 smoothpos;

        switch(m_interpolator.type){
        case SM_INTERPOLATOR_LINEAR:
          smoothpos = LERP(smoothfrom,smoothto,smoothfracc);
          break;
        case SM_INTERPOLATOR_CATMULLROM:
          smoothpos = CATMULLROM(smoothprev,smoothfrom,smoothto,smoothnext,smoothfracc);
          break;
        case SM_INTERPOLATOR_BEZIER:
          smoothpos = BEZIER(smoothfrom,smoothprev,smoothnext,smoothto,smoothfracc);
          break;
        }

        Point3 hit = LERP(curvepos,smoothpos,m_interpolator.weight);

        Point3 prevdir = (hit-spprev.pos);
        float prevdist = prevdir.Length();
        float lastdist = (hit-lastpos).Length();


        if (( prevdist >= stride &&
              lastdist >= stride*m_removethresh) ||
            ( lastdist <  stride*m_removethresh && 
             (prevdist + lastdist) > stride*m_removethresh*1.5 && 
              prevdist > lastdist)) 
        {
          StrokePoint& splast = m_strokepoints[m_lastpoint];
          splast.opos = splast.pos = hit;
          splast.size = GetSize(m_strokepoints[0].size);
          splast.flag = 0;
          AddPoint(lastpos,mat,GetSize(m_strokepoints[0].size));
          if (lastdist < stride * m_removethresh){
            index = oldcnt;
            break;
            // done
          }
        }
        runner+=fracc;
      }

      index++;
    }
  }

}

//#define StrokeDebugOp DebugOp
#define StrokeDebugOp __noop

void StrokeProcessor::EndLerpDeltas(const Point3 &endpt, const Matrix3 &endmat, const Point3& wnormal)
{
  Matrix3 endedgemat[2];
  Matrix3 startedgemat[2];

  Matrix3 endedgedeltamat[2];
  Matrix3 startedgedeltamat[2];

  Tab<Point3> deltas[2];
  
  deltas[0].SetCount(m_numdeltas);
  deltas[1].SetCount(m_numdeltas);

  {
    // facematrix
    // x = edge (ccw)
    // y = inside
    // z = face normal
    // deltamatrix
    // x = dir
    // y = edge
    // z = normal

    // config if wp ~= facenormal

    Point3 dir;
    Matrix3 smooth;

    dir = (m_strokepoints[1].pos - m_strokepoints[0].pos).Normalize();

    if (DotProd(m_startmat.GetRow(1),dir) > 0 && abs(DotProd(m_startmat.GetRow(2),dir)) < RAD_OF_45DEG ){
      // connect in face plane
      smooth.SetRow(0, m_startmat.GetRow(1));
      smooth.SetRow(1,-m_startmat.GetRow(0));
      smooth.SetRow(2, m_startmat.GetRow(2));
    }
    else if (DotProd(m_startmat.GetRow(2),dir) < 0){
      // connect top
      smooth.SetRow(0,-m_startmat.GetRow(2));
      smooth.SetRow(1,-m_startmat.GetRow(0));
      smooth.SetRow(2, m_startmat.GetRow(1));
    }
    else{
      // connect bottom
      smooth.SetRow(0, m_startmat.GetRow(2));
      smooth.SetRow(1,-m_startmat.GetRow(0));
      smooth.SetRow(2,-m_startmat.GetRow(1));
    }

    startedgemat[0].SetRow(0, dir);
    startedgemat[0].SetRow(1, CrossProd(wnormal,dir).Normalize());
    startedgemat[0].SetRow(2, wnormal);
    startedgemat[1] = smooth;

    startedgedeltamat[0] = Inverse(startedgemat[0]);
    startedgedeltamat[1] = Inverse(startedgemat[1]);

    dir = (endpt - m_strokepoints[m_lastpoint-1].pos).Normalize();

    if (DotProd(-endmat.GetRow(1),dir) > 0 && abs(DotProd(endmat.GetRow(2),dir)) < RAD_OF_45DEG ){
      // connect in face plane
      smooth.SetRow(0,-endmat.GetRow(1));
      smooth.SetRow(1, endmat.GetRow(0));
      smooth.SetRow(2, endmat.GetRow(2));
    }
    else if (DotProd(endmat.GetRow(2),dir) > 0){
      // connect bottom
      smooth.SetRow(0, endmat.GetRow(2));
      smooth.SetRow(1, endmat.GetRow(0));
      smooth.SetRow(2, endmat.GetRow(1));
    }
    else{
      // connect top
      smooth.SetRow(0,-endmat.GetRow(2));
      smooth.SetRow(1, endmat.GetRow(0));
      smooth.SetRow(2,-endmat.GetRow(1));
    }

    //smooth.RotateX(m_interpolator.tangentweight.w * DEG_TO_RAD);


    endedgemat[0].SetRow(0, dir);
    endedgemat[0].SetRow(1, CrossProd(wnormal,dir).Normalize());
    endedgemat[0].SetRow(2, wnormal);

    endedgemat[1] = smooth;

    //endedgemat.Orthogonalize();

    endedgedeltamat[0] = Inverse(endedgemat[0]);
    endedgedeltamat[1] = Inverse(endedgemat[1]);
    
  }

  for (int n = 0; n < m_numdeltas; n++){
    deltas[0][n] = startedgedeltamat[0].VectorTransform(m_deltasorig[n]);
    deltas[1][n] = startedgedeltamat[1].VectorTransform(m_deltasorig[n]);
  }


  // first pass from front
  Matrix3 lastmat[2];
  lastmat[0] = startedgemat[0];
  lastmat[1] = startedgemat[1];
  for (int i = 1; i < m_lastpoint; i++){

    const StrokePoint&  spprev = m_strokepoints[i-1];
    StrokePoint&            sp = m_strokepoints[i+0];
    const StrokePoint&  spnext = m_strokepoints[i+1];
    Matrix3 mat[2];

    Point3  scale(sp.size,sp.size,sp.size);

    for (int m = 0; m < 2; m++){
      // rotation minimal from start
      Point3  tonext = (spnext.pos-spprev.pos).Normalize();
      Point3  edgedir = CrossProd(lastmat[m].GetRow(2),tonext).Normalize();
      Point3  normal  = CrossProd(tonext,edgedir).Normalize();

      mat[m].Set(tonext,edgedir,normal,Point3(0.0f,0.0f,0.0f));

      lastmat[m] = mat[m];

      mat[m].Scale(scale);
      mat[m].SetTrans(sp.pos);
    }
    if (!(sp.flag & SM_STROKEFLAG_LOCK)){
      int begin =  0;
      int end   =  m_numdeltas;

      MNVert* __restrict vert = m_outmesh->V(sp.startvid);
      for (int i = begin; i < end; i++,vert++){
        vert->p = LERP( mat[0].PointTransform(deltas[0][i]),
                        mat[1].PointTransform(deltas[1][i]), m_interpolator.weight);
      }
    }
    for (int m = 0; m < 4; m++){
      sp.deltamat.SetRow(m,LERP(mat[0].GetRow(m),mat[1].GetRow(m),m_interpolator.weight));
    }
  }

  // second pass reversed

  for (int n = 0; n < m_numdeltas; n++){
    Point3  pt = m_outmesh->P(m_strokepoints[m_lastpoint].startvid + n) - endpt;
    deltas[0][n] = endedgedeltamat[0].VectorTransform(pt);
    deltas[1][n] = endedgedeltamat[1].VectorTransform(pt);
  }

  float fracc = 1.0f/float(m_lastpoint-1);
  float runner = fracc;

  lastmat[0] = endedgemat[0];
  lastmat[1] = endedgemat[1];
  for (int i = m_lastpoint-1; i > 0; i--){

    const StrokePoint&  spprev = m_strokepoints[i-1];
    StrokePoint&            sp = m_strokepoints[i+0];
    const StrokePoint&  spnext = m_strokepoints[i+1];
    Matrix3 mat[2];

    Point3  scale(sp.size,sp.size,sp.size);

    // rotation minimal from start
    for (int m = 0; m < 2; m++){
      Point3  tonext = (spnext.pos-spprev.pos).Normalize();
      Point3  edgedir = CrossProd(lastmat[m].GetRow(2),tonext).Normalize();
      Point3  normal  = CrossProd(tonext,edgedir).Normalize();

      mat[m].Set(tonext,edgedir,normal,Point3(0.0f,0.0f,0.0f));

      lastmat[m] = mat[m];

      mat[m].Scale(scale);
      mat[m].SetTrans(sp.pos);
    }

    float wt = runner;
    wt  = wt*wt*wt*(wt*(wt*6 - 15) + 10);

    if (!(sp.flag & SM_STROKEFLAG_LOCK)){
      // blend between front and back pass
      int begin =  0;
      int end   =  m_numdeltas;

      MNVert* __restrict vert = m_outmesh->V(sp.startvid);
      for (int v = begin; v < end; v++,vert++){
        Point3 pt = LERP( mat[0].PointTransform(deltas[0][v]),
                          mat[1].PointTransform(deltas[1][v]), m_interpolator.weight);
        vert->p = LERP(pt,vert->p,wt);
      }
    }

    for (int v = 0; v < 3; v++){
      sp.deltamat.SetRow(v,LERP(sp.deltamat.GetRow(v),LERP(mat[0].GetRow(v),mat[1].GetRow(v),m_interpolator.weight),wt));
    }

    runner += fracc;
  }

  endedgemat[0].SetRow(3,m_strokepoints[m_lastpoint].pos);
  endedgemat[1].SetRow(3,m_strokepoints[m_lastpoint].pos);
  m_strokepoints[m_lastpoint].size = 1;
  for (int m = 0; m < 4; m++){
    m_strokepoints[m_lastpoint].deltamat.SetRow(m,LERP(endedgemat[0].GetRow(m),endedgemat[1].GetRow(m),m_interpolator.weight));
  }

}

void StrokeProcessor::UpdateAll(int from, int to)
{
  SM_ASSERT(from <= to && from >= 0 && to >= 0 && to <= m_lastpoint && from <= m_lastpoint);

  for (int i = to; i >= from; i--){
    UpdatePoint(i);
  }
}

void StrokeProcessor::RelaxSlideVertices( int from, int to, int runs, float weight )
{
  float fracc = 1.0f/float(to-from+1);
  for (int r = 0; r < runs; r++){
    float wt = fracc;
    for (int i = from; i <= to; i++){
      StrokePoint* __restrict sp = &m_strokepoints[i];
      StrokePoint* spprev = sp-1;
      StrokePoint* spnext = sp+1;

      for (int v = 0; v < m_numdeltas; v++){
        Point3& self = m_outmesh->P(sp->startvid+v);
        Point3 spavg = (m_outmesh->P(spprev->startvid+v) + m_outmesh->P(spnext->startvid+v))*0.5f;
        self = LERP(self,spavg,weight*fracc);
      }
      wt += fracc;
    }
  }

}

void StrokeProcessor::EndFixTriangles(Tab<int> &endverts,int &firsttri, int &lasttri)
{
  // check last weldpair
  // and first + m_lastdelta
  int first = -1;
  int last = -1;

  if (m_lastpoint > 1){
    if (endverts[0] == m_wpairs[m_lastweld].ovid){
      first = m_wpairs[m_lastweld].avid;
    }
    if (endverts[m_lastdelta] == m_wpairs[m_lastweld].ovid){
      last = m_wpairs[m_lastweld].avid;
    }
  }
  if (first < 0 && endverts[0] == m_wpairs[0].ovid){
    first = m_wpairs[0].avid;
  }
  if (last < 0 && endverts[m_lastdelta] == m_wpairs[m_lastdelta].ovid){
    last = m_wpairs[m_lastdelta].avid;
  }

  if (last < 0 && first < 0)
    return;

  StrokePoint *sp = &m_strokepoints[m_lastpoint];
  // if triangles exist delete faces of last

  RemoveLastPointFaces();
  // and rebuild using overlapping indices
  int laststartid = m_strokepoints[m_lastpoint-1].startvid;
  for (int i = 0; i < m_numdeltas; i++){
    m_connectids[0][i] = laststartid+i;

    // override
    if (first >= 0 && i == 0)
      m_connectids[1][i] = first;
    else if (last >= 0 && i == m_lastdelta)
      m_connectids[1][i] = last;
    else
      m_connectids[1][i] = sp->startvid+i;
  }
  // can ignore closededge
  int cnt;
  sp->startfid = MNMesh_connectVAs(m_outmesh,m_connectids[0].Addr(0),m_connectids[1].Addr(0),m_numdeltas+m_closedring,cnt);
  for (int i = 0; i < m_lastdelta+m_closedring; i++){
    MNFace *face = m_outmesh->F(sp->startfid+i);
    face->smGroup = m_fattribs[i].smGroup;
    face->material = m_fattribs[i].material;
  }


  // also flag overlap vertices as dead
  // and tell external indices which weldpairs to skip
  if (first >= 0){
    firsttri = 0;
    m_outmesh->V(sp->startvid)->SetFlag(MN_DEAD);
  }
  if (last >= 0){
    lasttri = m_lastdelta;
    m_outmesh->V(sp->startvid+m_lastdelta)->SetFlag(MN_DEAD);
  }


}

BOOL StrokeProcessor::EndAtEdge(int endedgeid, const Point3 &curendpt)
{
  Point3  endpt;
  Tab<int> endedges;
  Tab<int> endverts;

  if (!GetEndEdges(endedgeid,endedges,endverts,endpt))
    return FALSE;

  Matrix3 endmat = MNMesh_getFaceMatrix(m_refmesh, m_refmesh->E(endedgeid)->f1,endedgeid,endpt);

  m_preendpoints = m_strokepoints;
  m_endmat  = endmat;

  if (m_lastpoint > 1){
    Point3 wnormal = m_objtminv.VectorTransform(m_wplane->GetNormal()).Normalize();

    if ((m_strokepoints[m_lastpoint].pos-m_strokepoints[m_lastpoint-1].pos).Length() <
      m_startstride*0.9f)
    {
      RemoveLastPoint();
    }

    m_strokepoints[m_lastpoint].pos = endpt;
    m_strokepoints[m_lastpoint].opos = endpt;

    EndLerpStrokepoints(endpt,endmat);
    
    for (int v = 0; v < m_numdeltas; v++){
      StrokePoint& sp = m_strokepoints[m_lastpoint];
      m_outmesh->P(sp.startvid + v) = m_refmesh->P(endverts[v]);
    }

    EndLerpDeltas(endpt,endmat,wnormal);
    if (m_lastpoint == 2){
      Point3 dir = (m_strokepoints[0].pos - endpt).Normalize();
      float absdot = abs(DotProd(dir,wnormal));
      // quick hack for short connections
      for (int i = 0; i < m_numdeltas; i++){
        m_outmesh->P( m_strokepoints[1].startvid + i) = LERP(
          m_outmesh->P( m_strokepoints[1].startvid + i),
          (m_outmesh->P( m_strokepoints[0].startvid + i ) + m_outmesh->P( m_strokepoints[2].startvid + i )) * 0.5f,
          powf(absdot,1.0/32.0f));
      }
    }
    else{
      //RelaxSlideVertices(max((m_lastpoint-3),1),m_lastpoint-1,2,0.5f);
    }

    DynamicWeld(0,m_lastpoint-1,m_dynweldmode);
    DynamicWeldFinalize();
  }

  // check whether last has triangles or not
  int firsttri = -1;
  int lasttri = -1;
  if (m_closedring){
    EndFixTriangles(endverts,firsttri,lasttri);
  }

  // weld last strokepoint
  StrokePoint *splast = &m_strokepoints[m_lastpoint];
  splast->flag |= SM_STROKEFLAG_LOCK;
  for (int i = 0; i < m_numdeltas; i++){
    // skip triangles
    if (i == firsttri || i == lasttri)
      continue;

    WeldPair wpair;
    wpair.avid = splast->startvid+i;
    wpair.ovid = endverts[i];

    splast->pos = endpt;
    m_wpairs.push_back(wpair);
    m_lastweld++;

    m_outmesh->V(wpair.avid)->p = m_refmesh->P(wpair.ovid);
  }

  m_outmesh->CollapseDeadStructs();
  m_outmesh->ClearFlag(MN_MESH_FILLED_IN);
  m_outmesh->FillInMesh();

  return TRUE;
}

void StrokeProcessor::EndEdge()
{
  DynamicWeldFinalize();

  if (m_refmesh->E(m_startedge)->GetFlag(MN_SEL)){
    StrokePoint *sp = &m_strokepoints[m_lastpoint];
    for (int i = 0; i < m_lastdelta+m_closedring; i++){

      int curvert = sp->startvid+i;
      int nextvert = sp->startvid+((i+1)%m_numdeltas);

      Tab<int>  &edgeids = m_outmesh->vedg[curvert];
      for (int n = 0; n < edgeids.Count(); n++){
        MNEdge *edge = m_outmesh->E(edgeids[n]);
        if (edge->OtherVert(curvert) == nextvert){
          //edge->SetFlag(MN_SEL);
          m_outmesh->SetEdgeSel(edgeids[n]);
          break;
        }
      }
    }
  }
}

void StrokeProcessor::EndFace()
{
  m_outmesh->CollapseDeadStructs();
  m_outmesh->ClearFlag(MN_MESH_FILLED_IN);
  m_outmesh->FillInMesh();

  // merge outer points with cap
  int lastvid = m_strokepoints[m_lastpoint].startvid;
  for (int i = 0; i < m_numdeltas; i++){
    m_outmesh->WeldBorderVerts(lastvid + i,m_capstartvid + i,&m_outmesh->P(lastvid + i));
  }

  // select cap faces
  for (int i = 0; i < m_numcapfaces; i++){
    m_outmesh->F(m_capstartfid + i)->SetFlag(MN_SEL);
  }

  m_outmesh->CollapseDeadStructs();
  m_outmesh->ClearFlag(MN_MESH_FILLED_IN);
  m_outmesh->FillInMesh();
}

void StrokeProcessor::RemoveLastTooClose(float sizemulti)
{
  if (m_lastpoint == 1) return;

  Point3 last   = m_strokepoints[m_lastpoint].pos;
  Point3 tolast = (last - m_strokepoints[m_lastpoint-1].pos);
  //float reflength = m_strokepoints[m_lastpoint].size * m_stridemulti * sizemulti;
  float reflength = m_startstride * sizemulti;

  // remove last point if too close
  if (tolast.Length() < reflength * 0.5){
    RemoveLastPoint();
  }
  tolast = (last - m_strokepoints[m_lastpoint-1].pos);
  if (sizemulti > 0.001f){
    // snap to multiplier
    float length = floorf(tolast.Length() / reflength);
    length = max(length,1.0f);

    length *= reflength / tolast.Length();
    m_strokepoints[m_lastpoint].pos = m_strokepoints[m_lastpoint-1].pos + (tolast * length);
  }

  UpdatePoint(m_lastpoint);
  DynamicWeld(m_lastpoint,m_lastpoint,m_dynweldmode);
}


void StrokeProcessor::EndAddPoints()
{
  const StrokePoint& lastpt = m_strokepoints[m_lastpoint];
  Point3  tohit = lastpt.pos-m_strokepoints[m_lastpoint-1].pos;
  float dist = tohit.Length();

  if (dist < 0.005)
    return;

  Point3 lastpos = lastpt.pos;
  float stride = m_startstride;
  float size = GetSize(lastpt.size);

  if (dist > stride){
    int from = AddInterPoints(lastpos,size,tohit,dist,stride,1000);
    UpdateAll(from,m_lastpoint);
    DynamicWeld(from,m_lastpoint,m_dynweldmode);
  }
}


void StrokeProcessor::End(int endedgeid, int endfaceid, const Point3 &endpt)
{
  Point3 wnormal = m_objtminv.VectorTransform(m_wplane->GetNormal()).Normalize();

  m_lastmode = m_mode;
  m_lastwplane = *m_wplane;

  m_endedge = endedgeid;
  m_endface = endfaceid;

  // make last edges selected as well
  if (!m_outmesh->GetFlag(MN_MESH_FILLED_IN)){
    m_outmesh->FillInMesh();
  }

  EndAddPoints();

  if (m_mode == STR_FREE){
    if ( !m_openInterpolation ){
      RemoveLastTooClose(m_removethresh);
    }
    m_startmat.SetRow(SM_EDGEMATROW_EDGE,     m_strokepoints[0].deltamat.VectorTransform(Point3(0,-1,0)).Normalize());
    m_startmat.SetRow(SM_EDGEMATROW_FACEXEDGE,m_strokepoints[0].deltamat.VectorTransform(Point3(1,0,0)).Normalize() );
    m_startmat.SetRow(SM_EDGEMATROW_FACE,     m_strokepoints[0].deltamat.VectorTransform(Point3(0,0,1)).Normalize());
    m_startmat.SetRow(SM_EDGEMATROW_POS,      m_strokepoints[0].opos);

    m_endmat.SetRow(SM_EDGEMATROW_EDGE,       m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(0,1,0)).Normalize());
    m_endmat.SetRow(SM_EDGEMATROW_FACEXEDGE,  m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(-1,0,0)).Normalize() );
    m_endmat.SetRow(SM_EDGEMATROW_FACE,       m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(0,0,1)).Normalize());
    m_endmat.SetRow(SM_EDGEMATROW_POS,        m_strokepoints[m_lastpoint].opos);

    if (m_openInterpolation){
      if (m_endoffset){
        m_strokepoints[m_lastpoint].pos = m_strokepoints[m_lastpoint].opos = ReprojectPoint(m_strokepoints[m_lastpoint].opos,m_endoffset);
        m_endmat.SetRow(SM_EDGEMATROW_POS, m_strokepoints[m_lastpoint].opos);
      }
      EndLerpStrokepoints(m_endmat.GetRow(SM_EDGEMATROW_POS),m_endmat,m_endoffset!=0);
      //UpdatePoint(m_lastpoint);
      //EndLerpDeltas(m_endmat.GetRow(SM_EDGEMATROW_POS),m_endmat,wnormal);
      UpdateAll(0,m_lastpoint);
    }

    float sz = 1.0f/m_strokepoints[0].size;
    Point3 pt = Point3(sz,-sz,sz);
    m_edgedeltamat.SetScale(pt);
    m_ended = true;
  }
  else {
    // check for connecting
    BOOL connected = FALSE;
    if (endedgeid >= 0){
      connected = EndAtEdge(endedgeid,endpt);
    }

    if (!connected){
      if ( !m_openInterpolation ){
        RemoveLastTooClose(m_removethresh);
      }

      m_endmat.SetRow(SM_EDGEMATROW_EDGE,       m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(0,1,0)).Normalize());
      m_endmat.SetRow(SM_EDGEMATROW_FACEXEDGE,  m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(-1,0,0)).Normalize() );
      m_endmat.SetRow(SM_EDGEMATROW_FACE,       m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(0,0,1)).Normalize());
      m_endmat.SetRow(SM_EDGEMATROW_POS,        m_strokepoints[m_lastpoint].opos);

      if (m_openInterpolation){
        if (m_endoffset){
          m_strokepoints[m_lastpoint].pos = m_strokepoints[m_lastpoint].opos = ReprojectPoint(m_strokepoints[m_lastpoint].opos,m_endoffset);
          m_endmat.SetRow(SM_EDGEMATROW_POS, m_strokepoints[m_lastpoint].opos);
        }
        EndLerpStrokepoints(m_endmat.GetRow(SM_EDGEMATROW_POS),m_endmat,m_endoffset!=0);
        //UpdatePoint(m_lastpoint);
        //EndLerpDeltas(m_endmat.GetRow(SM_EDGEMATROW_POS),m_endmat,wnormal);
        UpdateAll(1,m_lastpoint);
      }
    }

    if (m_startface >= 0)
      EndFace();
    else
      EndEdge();
  }

  m_outmesh->CollapseDeadStructs();
  m_outmesh->ClearFlag(MN_MESH_FILLED_IN);
  m_outmesh->FillInMesh();
  m_outmesh->InvalidateGeomCache();

  m_ended = true;
}

class EdgePartOfFaceSel : public MNMeshConVertCallBack
{
public:
  MNMesh*   m_mesh;
  EdgePartOfFaceSel (MNMesh *mesh) : m_mesh(mesh) {};

  BOOL IsEdgeLegal(int index){
    MNEdge *edge = m_mesh->E(index);
    return (  m_mesh->F(edge->f1)->GetFlag(MN_SEL) || (edge->f2 >= 0
        &&  m_mesh->F(edge->f2)->GetFlag(MN_SEL)));
  }
};

int StrokeProcessor::CorrectStartEdges(MNMesh* __restrict mesh, int seededgepos, Tab<int> &edges, Tab<int> &edgeverts)
{
  int cnt = edges.Count();
  int newfrom = 0;
  int newto = cnt-1;

  if (m_edgecorrthresh <= 0.0f) return seededgepos;
  float thresh = cosf(m_edgecorrthresh * DEG_TO_RAD);

  // seededge dir
  int seededgeid = edges[seededgepos];
  MNEdge *seededge = mesh->E(seededgeid);
  Matrix3 edgemat = MNMesh_getFaceMatrix(mesh,seededge->f1,seededgeid,Point3());

  for (int i = 0; i < cnt; i++){
    if (i == seededgepos) continue;

    Point3 dir = (mesh->P(edgeverts[i])-mesh->P(edgeverts[i+1])).Normalize();
    if (m_edgecorrinplane){
      Point3 normal = edgemat.GetRow(SM_EDGEMATROW_FACE);
      float corr = DotProd(dir,normal);
      if (fabs(corr) > 0.99f) // ~8 degress tolerance
        continue;

      dir = (dir - (normal*corr)).Normalize();
    }
    if (fabs(DotProd(dir,edgemat.GetRow(SM_EDGEMATROW_EDGE))) < thresh){
      if (i > seededgepos){
        newto = i-1;
        break;
      }
      else{
        newfrom = i+1;
      }
    }
  }

  if (newfrom != 0 || newto != cnt-1){
    Tab<int>  oldedges(edges);
    Tab<int>  oldverts(edgeverts);

    cnt = newto-newfrom+1;
    edges.SetCount(cnt);
    edgeverts.SetCount(cnt+1);

    // rebase data
    for (int i = 0; i < cnt; i++){
      edges[i] = oldedges[i+newfrom];
      edgeverts[i] = oldverts[i+newfrom];
    }
    edgeverts[cnt]= oldverts[newfrom+cnt];

    seededgepos -= newfrom;
  }

  return seededgepos;
}

int StrokeProcessor::GetStartEdges(int seededge, int seedface, const Point3 &brushpt,float brushsize, Tab<int> &edges, Tab<int> &edgeverts, MNMesh *overridemesh)
{
  MNMesh  *mesh = overridemesh ? overridemesh : m_refmesh;

  if( seedface >= 0){

    // first find vertices part of connected selected items
    m_convert->CheckMesh(mesh);
    m_convert->SeedSingle(SM_SO_FACE,seedface,1.0,Point3(),FALSE,FALSE,TRUE);

    EdgePartOfFaceSel einfacesel(mesh);
    m_convert->BuildAffectedRigid(true,&einfacesel);

    m_faceverts.SetSize(mesh->VNum());
    m_convert->AffectedToBitArray(m_faceverts,true);

    // get face selection
    m_faces.SetSize(mesh->FNum());
    mesh->getFaceSel(m_faces);

    // find edges that are on face border
    m_unprocedges.SetSize(mesh->ENum());
    m_unprocedges.ClearAll();
    BitArrayVertexToFacesBorder vtoborder(mesh,m_unprocedges,m_faces,TRUE);
    SM_BITARRAY_ENUMSET(m_faceverts,vtoborder,BitArrayVertexToFacesBorder);

    if (vtoborder.m_borderedge >= 0){
      bool reverse = false; //(mesh->F(mesh->E(vtoborder.m_borderedge)->f1)->GetFlag(MN_SEL));
      if (m_unprocedges[seededge]){
        vtoborder.m_borderedge = seededge;
      }
      m_refedgepos = MNMesh_findConnectedEdgeVerts(mesh,vtoborder.m_borderedge,m_unprocedges,edges,edgeverts,NULL,reverse,false);
      // fixme find closest edge on border
      float mindist = FLT_MAX;
      for (int i = 0; i < edges.Count(); i++){
        float dist = MNMesh_getEdgeDistance(mesh,edges[i],brushpt);
        if (dist < mindist){
          mindist = dist;
          m_refedgepos = i;
        }
      }
      return edges[m_refedgepos];
    }
    else{
      edges.SetCount(0);
      edgeverts.SetCount(0);
      return -1;
    }
  }
  else if (seededge >= 0){
    MNEdge *edge = mesh->E(seededge);
    BOOL  startselected = edge->GetFlag(MN_SEL);
    m_unprocedges.SetSize(mesh->ENum());

    if (startselected){
      // multi sweep possible
      mesh->getEdgeSel(m_unprocedges);
      // find connected series order them in v2-v1
      m_refedgepos = MNMesh_findConnectedEdgeVerts(mesh,seededge,m_unprocedges,edges,edgeverts);
    }
    else{
      // find brushsized
      m_unprocedges.SetAll();
      Point4 rangecheck(brushpt,brushsize);

      m_refedgepos = MNMesh_findConnectedEdgeVerts(mesh,seededge,m_unprocedges,edges,edgeverts,&rangecheck);
      // threshold correction
      m_refedgepos = CorrectStartEdges(mesh,m_refedgepos,edges,edgeverts);

      // disallow other edges
      m_unprocedges.ClearAll();
    }
    return seededge;
  }
  else{
    // shall never get here
    SM_ASSERT(0);
    return -1;
  }
}
BOOL  StrokeProcessor::OverlapsWithStart(int seededge)
{
  if (m_mode == STR_NONE || m_mode == STR_FREE) return FALSE;
  int ecnt = m_startedges.Count();
  for (int i = 0; i < ecnt; i++){
    if (m_startedges[i] == seededge)
      return TRUE;
  }
  return FALSE;
}
BOOL  StrokeProcessor::GetEndEdges(int seededge, Tab<int> &edges, Tab<int> &edgeverts, Point3 &anchor)
{
  if (!m_openendedges[seededge])
    return FALSE;

  MNEdge *endedge = m_refmesh->E(seededge);
  anchor = (m_refmesh->P(endedge->v1)-m_refmesh->P(endedge->v2))*m_startfracc + m_refmesh->P(endedge->v2);

  // speed up common case
  if (m_numdeltas == 2){
    edges.SetCount(1);
    edges[0] = seededge;
    edgeverts.SetCount(2);
    edgeverts[0] = endedge->v1;
    edgeverts[1] = endedge->v2;
    return TRUE;
  }
  else{
    BitArray allowed = m_openendedges;

    // flip before/after for reverse dir
    MNMesh_findConnectedEdgeVerts(m_refmesh,seededge,allowed,edges,edgeverts,NULL,true,true,
      m_refedgepos,m_lastdelta-m_refedgepos-1);

    BOOL closed = (edgeverts[0] == edgeverts[edgeverts.Count()-1]);
    if ((edgeverts.Count()-closed != m_numdeltas) || m_closedring != closed){
      return FALSE;
    }
    return TRUE;
  }
}


void StrokeProcessor::Clear()
{
  m_lastweld = -1;
  m_lastpoint = -1;
  m_lastdelta = -1;
  m_numdeltas   = 0;
  m_numcapverts = 0;
  m_mode = STR_NONE;
  m_railmode = RAIL_NONE;

  m_unprocedges.SetSize(0);
  m_openendedges.SetSize(0);
  m_faces.SetSize(0);
  m_faceverts.SetSize(0);
  m_fattribs.SetCount(0);
  m_delfaces.clear();
  m_wpairs.clear();
  m_dynwpairs[0].clear();
  m_dynwpairs[1].clear();
}

void StrokeProcessor::PrepStart(MNMesh *ref, MNMesh *out,WorkPlane *plane, Matrix3 objtm)
{
  m_refmesh = ref;
  m_outmesh = out;
  m_wplane = plane;
  m_objtm = objtm;
  m_objtminv = Inverse(objtm);

  m_lastweld = -1;
  m_lastpoint = -1;
  m_lastdelta = -1;
  m_numdeltas   = 0;
  m_numcapverts = 0;
  m_startmat.IdentityMatrix();

  m_unprocedges.SetSize(m_refmesh->ENum());
  m_openendedges.SetSize(m_refmesh->ENum());
  m_strokepoints.clear();
  m_wpairs.clear();
  m_delfaces.clear();
  m_dynwpairs[0].clear();
  m_dynwpairs[1].clear();
  m_lastdynamic[0] = 0;
  m_lastdynamic[1] = 0;
  m_mode = STR_NONE;
  m_railmode = RAIL_NONE;
}


void StrokeProcessor::Refine(StrokeProcessor& strokeproc)
{
  strokeproc.m_rotfactor=           m_rotfactor;
  strokeproc.m_rotfactorold=        m_rotfactorold;
  strokeproc.m_rotfree=             m_rotfree;
  strokeproc.m_dynweldfactor=       m_dynweldfactor;
  strokeproc.m_dynweldmode=         m_dynweldmode;
  strokeproc.m_relaxfactor=         m_relaxfactor;
  strokeproc.m_relaxruns=           m_relaxruns;
  strokeproc.m_removethresh=        m_removethresh;
  strokeproc.m_smooth=              m_smooth;
  strokeproc.m_smooththresh=        m_smooththresh;
  strokeproc.m_explicitstride=      m_explicitstride;
  strokeproc.m_interpolator=        m_interpolator;
  strokeproc.m_openInterpolation=   m_openInterpolation;
  strokeproc.m_lastwplane=          m_lastwplane;
  strokeproc.m_stridemulti=         m_stridemulti;
  strokeproc.m_endoffset=           m_endoffset;
  strokeproc.m_outmesh=             m_outmesh;
  
  *this = strokeproc;

  m_wplane = &m_lastwplane;

  int dirtystart = 1;

  RelaxPoints();

  // half/full rotate last points
  dirtystart = max(0,min(dirtystart,m_lastpoint-1));
  if (m_mode == STR_FREE){
    dirtystart = 0;
    // handle free rotation
    // rotate 
    m_deltas = m_freedeltas;
    Matrix3 rotmat = TransMatrix(-m_deltactr);
    float angle = m_rotfree * DEG_TO_RAD;
    rotmat.RotateX(angle);
    rotmat.Translate(m_deltactr);
    UpdateDeltas(rotmat);
  }
  UpdateAll(dirtystart,m_lastpoint);
  DynamicWeld(dirtystart,m_lastpoint,m_dynweldmode);
}


struct SmoothPair {
  DWORD smgroup;
  int   face;
};

void StrokeProcessor::DiscardChanges()
{
  MNMesh* __restrict mesh = m_refmesh;

  if (m_mode == STR_FACE){
    for (size_t i = 0; i < m_delfaces.size(); i++){
      //mesh->F(m_delfaces[i])->ClearFlag(MN_HIDDEN);
      //mesh->F(m_delfaces[i])->SetFlag(MN_SEL);
    }
  }
}

void StrokeProcessor::ApplyChanges(int subobj, SelectionMode selmode)
{
  MNMesh* __restrict mesh = m_refmesh;

  if (!m_ended){
    if (m_startface >= 0)
      EndFace();
    else
      EndEdge();

    m_outmesh->CollapseDeadStructs();
    m_outmesh->ClearFlag(MN_MESH_FILLED_IN);
    m_outmesh->FillInMesh();
    m_outmesh->InvalidateGeomCache();
  }

  int oldnumv = mesh->VNum();
  int oldnumf = mesh->FNum();
  int oldnume = mesh->ENum();

  m_lastmeshv = oldnumv;
  m_lastmeshf = oldnumf;
  m_lastmeshe = oldnume;

  for (int e = 0; e < oldnume; e++){
    mesh->SetEdgeSel(e,mesh->E(e)->GetFlag(MN_SEL) ? TRUE : FALSE);
  }

  if (selmode == SEL_ONLY){
    switch (subobj)
    {
    case SM_SO_VERTEX:
      mesh->ClearVFlags(MN_SEL);
      break;
    case SM_SO_EDGE:
    case SM_SO_BORDER:
      for (int i = 0; i < mesh->nume; i++){
        mesh->SetEdgeSel(i,FALSE);
      }
      break;
    case SM_SO_FACE:
    case SM_SO_ELEMENT:
      mesh->ClearFFlags(MN_SEL);
      break;
    }
  }
  {
    switch (subobj)
    {
    case SM_SO_VERTEX:
      for (int i = 0; i < m_outmesh->numv; i++){
        m_outmesh->V(i)->SetFlag(MN_SEL,selmode != SEL_NOCHANGE);
      }
      break;
    case SM_SO_EDGE:
    case SM_SO_BORDER:
      for (int i = 0; i < m_outmesh->nume; i++){
        m_outmesh->SetEdgeSel(i,selmode != SEL_NOCHANGE);
      }
      break;
    case SM_SO_FACE:
    case SM_SO_ELEMENT:
      for (int i = 0; i < m_outmesh->numf; i++){
        m_outmesh->F(i)->SetFlag(MN_SEL,selmode != SEL_NOCHANGE);
      }
      break;
    }
  }



  mesh->operator +=(*m_outmesh);

  if (m_delfaces.size() > 0){
    std::vector<int>::iterator itface = m_delfaces.begin();
    std::vector<int>::iterator itfaceend = m_delfaces.end();

    mesh->ClearFFlags (MN_USER);
    for (; itface < itfaceend; itface++){
      mesh->F(*itface)->SetFlag(MN_USER);
    }
    mesh->DeleteFlaggedFaces(MN_USER);
  }


  std::vector<WeldPair>::iterator it = m_wpairs.begin();
  std::vector<WeldPair>::iterator itend = m_wpairs.end();
  for (; it < itend; it++){
    bool ret = mesh->WeldBorderVerts(it->ovid,it->avid+oldnumv,NULL);
    if (!ret){
      ret = mesh->RemoveVertex(it->avid+oldnumv);
      ret = ret;
    }
  }

  mesh->FillInMesh();
  
  if (m_smooth){
    mesh->ClearFFlags (MN_USER);
    mesh->PropegateComponentFlags (MNM_SL_FACE, MN_USER, MNM_SL_FACE, MN_SEL);
    mesh->PropegateComponentFlags (MNM_SL_FACE, MN_SEL, MNM_SL_FACE, SM_STRIPFACE_FLAG);

#if 0
    mesh->ClearVFlags (MN_USER);
    mesh->PropegateComponentFlags (MNM_SL_VERTEX, MN_USER, MNM_SL_FACE, SM_STRIPFACE_FLAG);

    BitArray newSel;
    newSel.SetSize (mesh->numf);
    std::vector<SmoothPair> smgroups;
    for (int i=0; i<mesh->numf; i++) {
      int j;
      for (j=0; j<mesh->f[i].deg; j++) {
        if (mesh->v[mesh->f[i].vtx[j]].GetFlag (MN_USER)) break;
      }
      if (j<mesh->f[i].deg){
        if (!mesh->f[i].GetFlag(SM_STRIPFACE_FLAG)){
          SmoothPair spair;
          spair.face = i;
          spair.smgroup = mesh->f[i].smGroup;
          smgroups.push_back(spair);
        }
        newSel.Set (i);
      }
    }
    mesh->FaceSelect(newSel);
#endif
    mesh->AutoSmooth(m_smooththresh * DEG_TO_RAD,TRUE,FALSE);
    mesh->ClearFFlags(MN_SEL);
    mesh->PropegateComponentFlags (MNM_SL_FACE, MN_SEL, MNM_SL_FACE, MN_USER);
    mesh->ClearFFlags(MN_USER);
    mesh->ClearVFlags(MN_USER);
#if 0
    for (size_t i = 0; i < smgroups.size(); i++){
      mesh->f[smgroups[i].face].smGroup |= smgroups[i].smgroup;
    }
#endif
  }
  mesh->ClearFFlags(SM_STRIPFACE_FLAG);

}

StrokeProcessor::StrokeProcessor() : 
    m_lastpoint(-1)
  , m_stridemulti(1.0f)
  , m_lastdelta(-1)
  , m_relaxfactor(0.5f)
  , m_relaxruns(2)
  , m_rotfactor(0.0f)
  , m_edgecorrinplane(TRUE)
  , m_edgecorrthresh(45.0f)
  , m_removethresh(1.0f)
  , m_smooththresh(24.0f)
  , m_smooth(TRUE)
  , m_stretchconstrain(FALSE)
  , m_explicitstride(FALSE)
  , m_startedge(-1)
  , m_startface(-1)
  , m_mode(STR_NONE)
  , m_lastmode(STR_NONE)
  , m_dynweldfactor(0)
  , m_dynweldmode(WELD_SNAPOUTERVERTS)
  , m_deltastridemode(DELTASIZE_MAX)
  , m_rotfree(0)
  , m_endoffset(0)
{
  m_freedeltas.SetCount(2);
  m_freedeltas[0] = Point3(0.0f,1.0f,0.0f);
  m_freedeltas[1] = Point3(0.0f,-1.0f,0.0f);
}

void StrokeProcessor::ChangeOutput( MNMesh *out )
{
  if (m_outmesh){
    SM_ASSERT(m_outmesh->FNum() == out->FNum() && m_outmesh->VNum() == out->VNum() && m_outmesh->ENum() == out->ENum());
  }
  m_outmesh = out;
}

static void loadMatrix(Matrix3&mat, const Tab<float> &matrices, int idx)
{
  Point3 pts[4];

  for (int i = 0; i < 4; i++,idx+=3){
    pts[i].Set(matrices[idx+0],matrices[idx+1],matrices[idx+2]);
  }
  mat.Set(pts[0],pts[1],pts[2],pts[3]);
}

static inline void saveMatrix(Tab<float> &matrices, int idx, const Matrix3& mat)
{
  const float* matptr = (const float*)mat.GetAddr();
  for (int i = 0; i < 12; i++){
    matrices[idx+i] = matptr[i];
  }
}

void StrokeProcessor::ResetToStart()
{
  for (int i = 2; i < m_lastpoint+1; i++){
    if (m_strokepoints[i].startfid < 0)
      return;
    m_strokepoints[i].flag |= SM_STROKEFLAG_DEAD;
  }
  RemoveDeadPoints();
  m_outmesh->CollapseDeadStructs();
  m_outmesh->ClearFlag(MN_MESH_FILLED_IN);
  m_outmesh->FillInMesh();
  m_outmesh->InvalidateGeomCache();
  m_lastdynamic[0] = 0;
  m_lastdynamic[1] = 0;

  m_deltactr = m_deltactrorig;
  m_capdeltas = m_capdeltasorig;
  m_deltas = m_deltasorig;
  InitDeltaSize();
  if (m_mode == STR_FACE){
    UpdateDeltas(m_edgedeltamat);
  }
}

BOOL StrokeProcessor::SaveReplay( Tab<float>&replay ) const
{
  replay.SetCount(0);

  if (!CanReplay())
    return FALSE;

  int numPoints = (int)m_strokepoints.size();
  replay.SetCount(16 + 3*12 + numPoints*12);

  // version
  replay[0] = 1;
  replay[1] = (float)numPoints;
  replay[2] = (float)m_lastmode;
  replay[3] = m_startfracc;
  replay[4] = m_startrelative.x;
  replay[5] = m_startrelative.y;
  replay[6] = m_startrelative.z;
  replay[7] = m_deltasize;

  saveMatrix(replay,16+0,  m_startmat);
  saveMatrix(replay,16+12, m_endmat);
  saveMatrix(replay,16+24, m_edgedeltamat);

  for (int i = 0; i < numPoints; i++){
    saveMatrix(replay,16+(3+i)*12,m_strokepoints[i].deltamat);
  }

  return TRUE;
}

BOOL StrokeProcessor::LoadReplay( const Tab<float>&replay )
{
  if (replay.Count() < 4 || int(replay[0]) != 1 || replay.Count() != (int(replay[1])+3)*12+16 )
  {
    return FALSE;
  }

  int numPoints = int(replay[1]);
  m_lastmode    = (StrokeMode)int(replay[2]);
  m_startfracc    = replay[3];
  m_startrelative.x = replay[4];
  m_startrelative.y = replay[5];
  m_startrelative.z = replay[6];
  m_deltasize       = replay[7];

  loadMatrix(m_startmat,replay,16+0);
  loadMatrix(m_endmat, replay,16+12);
  loadMatrix(m_edgedeltamat,replay,16+24);

  m_strokepoints.resize(numPoints);
  for (int i = 0; i < numPoints; i++){
    loadMatrix(m_strokepoints[i].deltamat,replay,16+12*(3 + i));
  }
  return TRUE;
}

BOOL StrokeProcessor::ExtractReplay(int seededge, int seedface, MNMesh *mesh, WorkPlane *plane, const Matrix3& objtm)
{
  if (!MNMesh_isFaceSelected(mesh,seedface) || MNMesh_isFaceSelected(mesh,MNMesh_otherFace(mesh,seededge,seedface))) return FALSE;

  m_wplane = plane;
  m_objtm = objtm;
  m_objtminv = Inverse(objtm);

  MNEdge* edge = mesh->E(seededge);

  int startface = MNMesh_isFaceSelected(mesh,seedface) ? MNMesh_otherFace(mesh,seededge,seedface) : seedface;
  startface = startface < 0 ? seedface : startface;


  m_lastmode    = STR_FREE;
  m_startfracc  = 0.5;
  m_startrelative = Point3(0,0,0);
  m_deltasize   = MNMesh_getEdgeLength(mesh,seededge);

  Point3 point  = (mesh->P(edge->v1) + mesh->P(edge->v2)) * 0.5f;
  Matrix3 mat   = MNMesh_getFaceMatrix(mesh,startface,seededge,point);
  m_startmat    = mat;

  bool startfirst = edge->f1 == startface;

  if (startface == seedface || !startfirst){
    m_startmat.SetRow(0,-mat.GetRow(0));
    m_startmat.SetRow(1,-mat.GetRow(1));
    startfirst = !startfirst;
  }

  m_edgedeltamat.IdentityMatrix();

  m_strokepoints.resize(0);
  m_lastpoint = -1;
  
  int faceid = seedface;
  int edgeid = seededge;
  int lastfaceid = seedface;
  while (true){
    edge = mesh->E(edgeid);
    point = (mesh->P(edge->v1) + mesh->P(edge->v2)) * 0.5f;

    float scale = MNMesh_getEdgeLength(mesh,edgeid)/m_deltasize;
    Point3 scalefactor(scale,scale,scale);

    bool isfirst = edge->f1 == lastfaceid;

    int sign = isfirst == startfirst ? -1 : 1;

    StrokePoint spnew;
    mat = MNMesh_getFaceMatrix(mesh,lastfaceid,edgeid,point);
    mat.Scale(scalefactor);
    spnew.deltamat.SetRow(SM_EDGEMATROW_EDGE,       mat.VectorTransform(Point3(0,-sign,0)));
    spnew.deltamat.SetRow(SM_EDGEMATROW_FACEXEDGE,  mat.VectorTransform(Point3(sign,0,0)));
    spnew.deltamat.SetRow(SM_EDGEMATROW_FACE,       mat.VectorTransform(Point3(0,0,1)));
    spnew.deltamat.SetRow(SM_EDGEMATROW_POS, point);
    spnew.opos = spnew.pos = point;
    m_strokepoints.push_back(spnew);
    m_lastpoint++;

    if (faceid < 0 || !MNMesh_isFaceSelected(mesh,faceid))
      break;

    lastfaceid = faceid;
    edgeid = MNMesh_nextEdgeRing(mesh,edgeid,faceid);
    edge = mesh->E(edgeid);
    faceid = edge->OtherFace(faceid);
  }

  if (startface != seedface){
    m_lastmode = STR_EDGE;
    m_edgedeltamat = GetEdgeMat(m_strokepoints[1].pos, m_strokepoints[0].pos, mesh->GetFaceNormal(seedface,TRUE));
  }

  m_endmat.SetRow(SM_EDGEMATROW_EDGE,       m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(0,1,0)).Normalize());
  m_endmat.SetRow(SM_EDGEMATROW_FACEXEDGE,  m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(-1,0,0)).Normalize() );
  m_endmat.SetRow(SM_EDGEMATROW_FACE,       m_strokepoints[m_lastpoint].deltamat.VectorTransform(Point3(0,0,1)).Normalize());
  m_endmat.SetRow(SM_EDGEMATROW_POS,        m_strokepoints[m_lastpoint].opos);


  return TRUE;

}

void StrokeProcessor::Replay( const StrokeProcessor& replay, bool flipSide, bool scale )
{
  if (m_mode == STR_FREE){
    m_startmat.SetRow(SM_EDGEMATROW_EDGE,     m_strokepoints[0].deltamat.VectorTransform(Point3(0,-1,0)).Normalize());
    m_startmat.SetRow(SM_EDGEMATROW_FACEXEDGE,m_strokepoints[0].deltamat.VectorTransform(Point3(1,0,0)).Normalize() );
    m_startmat.SetRow(SM_EDGEMATROW_FACE,     m_strokepoints[0].deltamat.VectorTransform(Point3(0,0,1)).Normalize());
    m_startmat.SetRow(SM_EDGEMATROW_POS,      m_strokepoints[0].opos);
    m_edgedeltamat.IdentityMatrix();
  }

  // must not modify original m_startmat!
  Matrix3 startmat = m_startmat;

  float posscale = 1.0f;

  if (m_mode == STR_EDGE){
    Matrix3 move(1);
    MNEdge* edge = m_refmesh->E(m_startedge);
    float sz = replay.m_lastmode != STR_FREE ? replay.m_startfracc : 0.5f;
    Point3 newctr = m_refmesh->P(edge->v1) + (m_refmesh->P(edge->v2)-m_refmesh->P(edge->v1)) * sz;
    move.SetTranslate(  m_startmat.GetRow(3) - newctr);

    UpdateDeltas(move);
    startmat.SetRow(3,newctr);

    m_strokepoints[0].opos = newctr;
    m_strokepoints[0].pos  = newctr;

  }
  else if (m_mode == STR_FACE && replay.m_lastmode == STR_FACE){
    Matrix3 move(1);

    Matrix3 startinv = Inverse(m_startmat);
    Box3 bbox;
    bbox.Init();
    for (int i = 0; i < m_numdeltas; i++){
      bbox += startinv.VectorTransform(m_deltasorig[i]);
    }

    Point3 newctr = replay.m_startrelative * bbox.Width() + bbox.Min();
    newctr.z = 0.0f;
    newctr = startmat.PointTransform(newctr);


    Point3 delta = m_startmat.GetRow(3) - newctr;
    delta = m_edgedeltamat.VectorTransform(delta);
    move.SetTranslate( delta );
    UpdateDeltas(move);

    startmat.SetRow(3,newctr);
    m_strokepoints[0].opos = newctr;
    m_strokepoints[0].pos  = newctr;
  }

  if (scale && (m_mode == STR_FREE || m_mode == STR_EDGE))
  {
    posscale = m_deltasize / replay.m_deltasize;
  }

  Matrix3 flip(1);
  flip.SetRow(0,flipSide ? -flip.GetRow(0) : flip.GetRow(0));

  Matrix3 scaledstartmat = replay.m_startmat;
  scaledstartmat.SetTrans(scaledstartmat.GetTrans() * posscale );

  Matrix3 convert = Inverse(scaledstartmat) * flip * startmat;

  m_deltadelay = -2;
  m_startfracc = replay.m_startfracc;
  
  if (m_mode != STR_FACE){
    if (m_mode != STR_FREE){
      if (replay.m_lastmode != STR_FREE){
        m_edgedeltamat = ((Inverse(startmat)) * replay.m_startmat) * replay.m_edgedeltamat;
      }
      else{
        Matrix3 newdelta = startmat;
        Point3 tmp = newdelta.GetRow(1);
        newdelta.SetRow(1,newdelta.GetRow(0));
        newdelta.SetRow(0,tmp);

        m_edgedeltamat = Inverse(newdelta) * replay.m_edgedeltamat;
      }
    }
    else{
      float sz = m_strokepoints[0].size;
      Matrix3 scale;
      if (replay.m_lastmode == STR_EDGE){
        //m_edgedeltamat = (scale * replay.m_edgedeltamat) * Inverse(replay.m_startmat);
        scale.SetScale(Point3(sz,sz,sz));
        m_edgedeltamat = (scale * m_edgedeltamat);
      }
      else if (replay.m_lastmode == STR_FACE){
        //m_edgedeltamat = (scale * replay.m_edgedeltamat) * Inverse(replay.m_startmat);
        scale.SetScale(Point3(sz,-sz,-sz));
        m_edgedeltamat = (scale * m_edgedeltamat);
      }
      else{
        scale.SetScale(Point3(sz,-sz,sz));
        m_edgedeltamat = (scale * m_edgedeltamat) * replay.m_edgedeltamat;
      }
    }
    m_edgedeltamat.SetRow(3,Point3(0,0,0));
    UpdateDeltas(m_edgedeltamat);
  }

  flip.IdentityMatrix();
  flip.SetRow(1,flipSide ? -flip.GetRow(1) : flip.GetRow(1));
  UpdateDeltas(flip);

  // copy over stroke points
  // and convert coordinates/matrices according to m_startmat

  int newPoints = replay.m_strokepoints.size() - m_strokepoints.size();
  Matrix3 mat(1);

  for (int i = 0; i < newPoints; i++){
    AddPointRaw();
  }

  Point3 unit(1,1,1);
  unit.Normalize();

  for (int i = 1; i < replay.m_strokepoints.size(); i++){
    StrokePoint &spnew = m_strokepoints[i];
    const StrokePoint &spold = replay.m_strokepoints[i];
    Matrix3 scaleddelta = spold.deltamat;
    scaleddelta.SetTrans( scaleddelta.GetTrans() * posscale );
    
    spnew.deltamat = scaleddelta * convert;
    spnew.opos  = spnew.pos = spnew.deltamat.PointTransform(Point3(0,0,0));
    spnew.size = spnew.deltamat.VectorTransform(unit).Length();
    UpdateVertices(spnew, spnew.deltamat);
  }

  DynamicWeld(0,m_lastpoint,m_dynweldmode);
  DynamicWeldFinalize();

  // make last edges selected as well
  m_outmesh->CollapseDeadStructs();
  m_outmesh->ClearFlag(MN_MESH_FILLED_IN);
  m_outmesh->FillInMesh();
  m_outmesh->InvalidateGeomCache();
}





