/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "meshlooping.h"

//////////////////////////////////////////////////////////////////////////
//

#ifdef SM_MNMESHLOOP_MAXLIKE
  #define MNMeshLoop_nextEdgeLoop(mesh,edge,convert)  MNMesh_nextEdgeLoopMax(mesh,edge,convert)
  #define MNMeshLoop_nextEdgeRing(mesh,edge,conface)  MNMesh_nextEdgeRingMax(mesh,edge,conface)
#else
  #define MNMeshLoop_nextEdgeLoop(mesh,edge,convert)  MNMesh_nextEdgeLoop(mesh,edge,convert)
  #define MNMeshLoop_nextEdgeRing(mesh,edge,conface)  MNMesh_nextEdgeRing(mesh,edge,conface)
#endif



//////////////////////////////////////////////////////////////////////////
// MNMeshLoopAdvancerVertex

BOOL MNMeshLoopAdvancer::Advance(MNMeshLoopFront* front, MNMeshLoopItem* item, int itemindex, int wave, BitArray &finalsel)
{
  if (front->nextid < 0 || (front->crossed && finalsel[front->nextid]))
    return TRUE;


  int newid = front->nextid;
  item->wave = wave;
  item->previndex = front->previndex;
  item->id = newid;
  item->pos = GetPos(newid);
  front->previndex = itemindex;
  front->previd = newid;

  front->crossed = finalsel[newid];
  finalsel.Set(newid);

  // check for next connector
  front->connector = GetNextConnector(front->connector,front->contype,newid);
  //front->connector = front->connector == newconnector ? -1 : newconnector;
  front->nextid = front->connector < 0 ? -1 : GetNextID(front->connector,front->contype,newid);

  return FALSE;
}

BOOL MNMeshLoopAdvancer::Advance(MNMeshLoopFront* front, MNMeshLoopItem* item)
{
  if (front->nextid < 0)
    return TRUE;


  int newid = front->nextid;
  item->previndex = front->previndex;
  item->id = newid;
  item->pos = GetPos(newid);
  front->previd = newid;


  // check for next connector
  front->connector = GetNextConnector(front->connector,front->contype,newid);
  //front->connector = front->connector == newconnector ? -1 : newconnector;
  front->nextid = front->connector < 0 ? -1 : GetNextID(front->connector,front->contype,newid);

  return FALSE;
}

MNMeshLoopFront* MNMeshLoopAdvancer::FindDirectedFront(Tab<MNMeshLoopFront> &fronts, Point3 &start, Point3 &dir, GraphicsWindow *gw)
{
  MNMeshLoopFront *front;
  MNMeshLoopItem  item;
  Point3 test;

  float maxcos = -1.0f;
  float anglecos;
  int   minout = -1;

  // then advance each direction once and test for best angle
  int outcount = fronts.Count();
  for (int o = 0; o < outcount; o++){
    front = fronts.Addr(o);

    // loop ended
    if (Advance(front,&item))
      continue;

    // loop worked, check angle
    gw->transPoint(&item.pos,&test); test.z = 0.0f;

    anglecos = DotProd((test-start).Normalize(),dir);
    if (anglecos > maxcos){
      minout = o;
      maxcos = anglecos;
    }
  }

  return (minout < 0) ? NULL : fronts.Addr(minout);
}

//////////////////////////////////////////////////////////////////////////
// MNMeshLoopAdvancerVertex

int MNMeshLoopAdvancerVertex::SetupFront(int startid, Tab<MNMeshLoopFront> &fronts)
{
  // first previous is root
  // set initial connectors
  Tab<int> &connected = m_mesh->vedg[startid];
  int outcount = connected.Count();
  // for each direction
  fronts.SetCount(outcount);
  for (int o = 0; o < outcount; o++){
    MNMeshLoopFront *advance = fronts.Addr(o);

    int nextid = m_mesh->E(connected[o])->OtherVert(startid);
    advance->previndex = 0;
    advance->nextid = nextid;
    advance->connector = connected[o];
    advance->crossed = FALSE;
  }

  return outcount;
}

Point3 MNMeshLoopAdvancerVertex::GetPos(int id)
{
  return m_mesh->P(id);
}
int    MNMeshLoopAdvancerVertex::GetNextID(int connector, int contype, int curid)
{
  return m_mesh->E(connector)->OtherVert(curid);
}
int    MNMeshLoopAdvancerVertex::GetNextConnector(int curconnector, int contype, int curid)
{
  return MNMeshLoop_nextEdgeLoop(m_mesh,curconnector,curid);
}

BOOL  MNMeshLoopAdvancerVertex::AreConnected(int a, int b, int &con)
{
  Tab<int> &outedges = m_mesh->vedg[a];
  int outcnt = outedges.Count();
  for (int i = 0; i < outcnt; i++){
    if (MNEdge_connected(m_mesh->E(outedges[i]),b)){
      con = outedges[i];
      return TRUE;
    }

  }
  return FALSE;
}

void MNMeshLoopAdvancerVertex::Select(int id, bool state)
{
  m_mesh->V(id)->SetFlag(MN_SEL,state);
}
void MNMeshLoopAdvancerVertex::GetSelected(BitArray &sel)
{
  m_mesh->getVertexSel(sel);
}
void MNMeshLoopAdvancerVertex::SetSelected(BitArray &sel)
{
  m_mesh->VertexSelect(sel);
}
int  MNMeshLoopAdvancerVertex::GetCount()
{
  return m_mesh->VNum();
}
//////////////////////////////////////////////////////////////////////////
// MNMeshLoopAdvancerEdge
// TODO
int MNMeshLoopAdvancerEdge::SetupFront(int startid, Tab<MNMeshLoopFront> &fronts)
{
  // first previous is root
  // set initial connectors
  MNEdge *edge = m_mesh->E(startid);

  fronts.SetCount(0);
  int outcount = 0;
  for (int o = 0; o < 4; o++){
    if (m_disableFlag & 1<<o) continue;

    int connector = -1;
    int contype;
    int nextid;

    if (o < 2){
      // loop
      contype = 0;
      connector = o ? edge->v1 : edge->v2;
      nextid = MNMeshLoop_nextEdgeLoop(m_mesh,startid,connector);
    }
    else{
      // ring
      contype = 1;
      connector = o%2 ? edge->f1 : edge->f2;
      nextid = connector >= 0 ? MNMeshLoop_nextEdgeRing(m_mesh,startid,connector) : -1;
    }

    if (nextid >= 0){
      MNMeshLoopFront advance;
      advance.previndex = 0;
      advance.nextid = nextid;
      advance.connector = connector;
      advance.contype = contype;
      advance.crossed = FALSE;

      outcount++;

      fronts.Append(1,&advance);
    }
  }

  return outcount;
}

Point3 MNMeshLoopAdvancerEdge::GetPos(int id)
{
  MNEdge *edge = m_mesh->E(id);

  return (m_mesh->P(edge->v1)+m_mesh->P(edge->v2))*0.5f;
}
int    MNMeshLoopAdvancerEdge::GetNextID(int connector, int contype, int curid)
{
  return contype ? MNMeshLoop_nextEdgeRing(m_mesh,curid,connector) : MNMeshLoop_nextEdgeLoop(m_mesh,curid,connector);
}
int    MNMeshLoopAdvancerEdge::GetNextConnector(int curconnector, int contype, int curid)
{
  return contype ? m_mesh->E(curid)->OtherFace(curconnector) : m_mesh->E(curid)->OtherVert(curconnector);
}
BOOL  MNMeshLoopAdvancerEdge::AreConnected(int a, int b, int &con)
{
  MNEdge *edgea = m_mesh->E(a);
  MNEdge *edgeb = m_mesh->E(b);

  if( m_disableFlag == ~3 && (
      // if self is border, keep on border
      (edgea->f2 <= 0 && edgeb->f2 >= 0
#if 0
      // if self not border, ensure edge is part of different face
       ||(edgea->f2 > 0 && 
        ( edgea->f2 == edgeb->f1 ||
          edgea->f2 == edgeb->f2 ||
          edgea->f1 == edgeb->f1 ||
          edgea->f1 == edgeb->f2))
#endif
       )
      )
    )
  {
    return FALSE;
  }

  if (edgea->v1 == edgeb->v1 ||
    edgea->v1 == edgeb->v2)
  {
    con = edgea->v1;
    return TRUE;
  }

  if (edgea->v2 == edgeb->v1 ||
    edgea->v2 == edgeb->v2)
  {
    con = edgea->v2;
    return TRUE;
  }

  return FALSE;
}
void MNMeshLoopAdvancerEdge::Select(int id, bool state)
{
  m_mesh->SetEdgeSel(id,state ? 1 : 0);
  //m_mesh->E(id)->SetFlag(MN_SEL,state);
}
void MNMeshLoopAdvancerEdge::GetSelected(BitArray &sel)
{
  m_mesh->getEdgeSel(sel);
}
void MNMeshLoopAdvancerEdge::SetSelected(BitArray &sel)
{
  m_mesh->EdgeSelect(sel);
}
int  MNMeshLoopAdvancerEdge::GetCount()
{
  return m_mesh->ENum();
}
//////////////////////////////////////////////////////////////////////////
// MNMeshLoopAdvancerFace
int MNMeshLoopAdvancerFace::SetupFront(int startid, Tab<MNMeshLoopFront> &fronts)
{
  // first previous is root
  // set initial connectors
  MNFace *face = m_mesh->F(startid);

  int outcount = face->deg;
  // for each direction
  fronts.SetCount(outcount);
  for (int o = 0; o < outcount; o++){
    MNMeshLoopFront *advance = fronts.Addr(o);

    int nextid = m_mesh->E(face->edg[o])->OtherFace(startid);
    advance->previndex = 0;
    advance->nextid = nextid;
    advance->connector = face->edg[o];
    advance->crossed = FALSE;
  }

  return outcount;
}

Point3 MNMeshLoopAdvancerFace::GetPos(int id)
{
  Point3 sum(0,0,0);
  MNFace *face = m_mesh->F(id);

  int deg = face->deg;
  for (int i = 0; i < deg; i++){
    sum += m_mesh->P(face->vtx[i]);
  }

  sum /= (float)deg;

  return sum;
}
int    MNMeshLoopAdvancerFace::GetNextID(int connector, int contype, int curid)
{
  return m_mesh->E(connector)->OtherFace(curid);
}
int    MNMeshLoopAdvancerFace::GetNextConnector(int curconnector, int contype, int curid)
{
  return MNMeshLoop_nextEdgeRing(m_mesh,curconnector,curid);
}
BOOL  MNMeshLoopAdvancerFace::AreConnected(int a, int b, int &con)
{
  MNFace *face = m_mesh->F(a);
  int outcnt = face->deg;
  for (int i = 0; i < outcnt; i++){
    if (m_mesh->E(face->edg[i])->OtherFace(a) == b){
      con = face->edg[i];
      return TRUE;
    }

  }
  return FALSE;
}
void MNMeshLoopAdvancerFace::Select(int id, bool state)
{
  m_mesh->F(id)->SetFlag(MN_SEL,state);
}
void MNMeshLoopAdvancerFace::GetSelected(BitArray &sel)
{
  m_mesh->getFaceSel(sel);
}
void MNMeshLoopAdvancerFace::SetSelected(BitArray &sel)
{
  m_mesh->FaceSelect(sel);
}
int  MNMeshLoopAdvancerFace::GetCount()
{
  return m_mesh->FNum();
}
//////////////////////////////////////////////////////////////////////////
// MNMeshLoop

/*
  start from main id, and then check connected which are also selected
  operate in region grow
  when loopitem is added remove item from selection
*/

#define MESHLOOP_ALLOC_ITEMCOUNT 32

void MNMeshLoop::Build(MNMeshLoopAdvancer &adv, int startid, BitArray &finalsel)
{
  // prepare Loopitems
  m_items.SetCount(MESHLOOP_ALLOC_ITEMCOUNT);
  int allocated = MESHLOOP_ALLOC_ITEMCOUNT;

  // add ourself
  int itemcnt = 0;
  int wave = 1;

  MNMeshLoopItem *item = m_items.Addr(0);
  item->wave = 0;
  item->distance = 0.0f;
  item->id = startid;
  item->pos = adv.GetPos(startid);
  item->prev = NULL;
  finalsel.Set(startid);
  item++;
  itemcnt++;

  Tab<MNMeshLoopFront> fronts;
  int outcount = adv.SetupFront(startid,fronts);

  // then advance each direction
  int added = TRUE;
  while(added){
    added = FALSE;
    for (int o = 0; o < outcount; o++){
      MNMeshLoopFront *front = fronts.Addr(o);

      // loop ended
      if (adv.Advance(front,item,itemcnt,wave,finalsel))
        continue;

      item++;
      itemcnt++;

      // expand memory (we could use append but well no clue how it resizes)
      if (itemcnt%MESHLOOP_ALLOC_ITEMCOUNT == 0){
        m_items.SetCount(itemcnt+MESHLOOP_ALLOC_ITEMCOUNT);
        item = m_items.Addr(itemcnt);
      }
      added = TRUE;
    }
    wave += 1;
  }

  m_numitems = itemcnt;
}

void MNMeshLoop::Clear()
{
  m_numitems = 0;
  m_items.SetCount(0);
}
void MNMeshLoop::LinkPrev()
{

  MNMeshLoopItem  *browse = m_items.Addr(0);
  MNMeshLoopItem  *end = browse+m_numitems;

  // first has NULL already applied
  browse++;

  while(browse < end){
    browse->prev = m_items.Addr(browse->previndex);
    //browse->distance = browse->prev->distance + (browse->prev->pos-browse->pos).Length();
    browse++;
  }
}

//////////////////////////////////////////////////////////////////////////
// MNMesh helpers

// -----------
// quad based
int MNMesh_nextEdgeLoop(MNMesh *mesh, int eid, int convert, BOOL aroundcorners)
{
  MNEdge* edge = mesh->E(eid);
  MNFace* face = mesh->F(edge->f1);
  int testface = edge->f1;

  if (face->deg != 4)
    return -1;

  int outedgecnt = mesh->vedg[convert].Count();

  if (edge->f2 >= 0 && outedgecnt != 4)
    return -1;

  // check wheter the edge before or after us
  // contains the needed vertex
  int conedge = -1;
  for (int d = 0; d < face->deg ; d++){
    if (face->edg[d] == eid){
      int test = face->edg[(d+1)%face->deg];
      edge = mesh->E(test);
      if (edge->v1 == convert || edge->v2 == convert){
        conedge = test;
        break;
      }

      test = face->edg[(d-1+face->deg)%face->deg];
      edge = mesh->E(test);
      if (edge->v1 == convert || edge->v2 == convert){
        conedge = test;
        break;
      }
    }
  }
  if (conedge < 0)
    return -1;

  if (aroundcorners && outedgecnt == 2)
    return conedge;

  testface = edge->OtherFace(testface);
  if (testface < 0)
    return -1;

  face = mesh->F(testface);
  if (face->deg != 4)
    return -1;

  // find edge that connects to convert
  for (int d = 0; d < face->deg ; d++){
    eid = face->edg[d];
    if (eid != conedge && (edge = mesh->E(eid)) != NULL &&
      (edge->v1 == convert || edge->v2 == convert))
    {
      return eid;
      break;
    }
  }

  return -1;
}

int MNMesh_nextEdgeRing(MNMesh *mesh, int edge, int conface)
{
  MNFace *face = mesh->F(conface);
  if (face->deg != 4) return -1;
  for (int i=0; i < 4; i++){
    if (face->edg[i] == edge)
      return face->edg[(i+2)%4];
  }
  return -1;
}

int MNMesh_nextVertexLoop(MNMesh *mesh, int vert, int conedge)
{
  // same as edge loop
  MNEdge *edge = mesh->E(conedge);
  return edge->OtherVert(vert);
}

int MNMesh_nextFaceLoop(MNMesh*mesh, int face, int conedge)
{
  return mesh->E(conedge)->OtherFace(face);
}

// ----------
// 3dsmax like

int MNMesh_nextEdgeRingMax(MNMesh *mesh, int edgeid, int faceid)
{
  MNFace *face = mesh->F(faceid);
  int deg = face->deg;
  if (deg < 4)
    return -1;

  for (int i=0; i < deg; i++){
    if (face->edg[i] == edgeid){
      int eidnext = face->edg[(i+2)%deg];
      int eidprev = face->edg[(i-2+deg)%deg];

      int nface = mesh->E(eidnext)->OtherFace(faceid);
      eidnext = mesh->F(nface)->deg == 4 ? eidnext : -1;

      nface = mesh->E(eidprev)->OtherFace(faceid);
      eidprev = mesh->F(nface)->deg == 4 ? eidprev : -1;

      // when both are connected to quads, stop
      // else pick one that is a quad

      return (eidprev >= 0 && eidnext >= 0) ? -1 :
        ((eidnext >= 0) ? eidnext : eidprev);
    }
  }
  return -1;
}
int MNMesh_findConnectedFaceEdge(MNMesh *mesh, MNFace *face, int eid, int convert, MNEdge **conedge)
{
  for (int d = 0; d < face->deg ; d++){
    if (face->edg[d] == eid){
      int test = face->edg[(d+1)%face->deg];
      MNEdge *edge = mesh->E(test);
      if (edge->v1 == convert || edge->v2 == convert){
        *conedge = edge;
        return test;
      }

      test = face->edg[(d-1+face->deg)%face->deg];
      edge = mesh->E(test);
      if (edge->v1 == convert || edge->v2 == convert){
        *conedge = edge;
        return test;
      }
    }
  }
  return -1;
}

int MNMesh_nextEdgeLoopMax(MNMesh *mesh, int eid, int convert)
{
  MNEdge* edge = mesh->E(eid);
  MNFace* face = mesh->F(edge->f1);
  int testface = edge->f1;

  if (mesh->vedg[convert].Count() != (edge->f2 < 0 ? 3 : 4))
    return -1;

  // check wheter the edge before or after us
  // contains the needed vertex
  int conedge = MNMesh_findConnectedFaceEdge(mesh,face,eid,convert,&edge);

  if (conedge < 0){
    return -1;
  }

  testface = edge->OtherFace(testface);
  if (testface < 0)
    return -1;

  face = mesh->F(testface);
  // find new conedge
  conedge = MNMesh_findConnectedFaceEdge(mesh,face,conedge,convert,&edge);
  testface = edge->OtherFace(testface);

  // if connector face or opposite face of new edge arent quads, then
  // discard
  if (face->deg != 4 && (testface < 0 || mesh->F(testface)->deg != 4))
    return -1;

  return conedge;
}

void MNMesh_selectVertexLoops(MNMesh *mesh, int startid, BitArray &vertexloops)
{
  BitArray secondarysel(mesh->ENum());
  BitArray connectededges(mesh->ENum());

  vertexloops.SetSize(mesh->VNum());

  mesh->getEdgeSel(secondarysel);
  // convert to Edge
  Tab<int> &connected = mesh->vedg[startid];
  for (int e=0; e < connected.Count(); e++){
    connectededges.Set(connected[e]);
  }
  // loop edges
  mesh->SelectEdgeLoop(connectededges);
  // convert back to vertex
  MNEdge *edge = mesh->e;
  MNEdge *lastedge = edge+mesh->ENum();
  int eid = 0;
  while (edge < lastedge){
    if (connectededges[eid]){
      vertexloops.Set(edge->v1);
      vertexloops.Set(edge->v2);
    }

    edge++;
    eid++;
  }
  // select those vertices
  mesh->VertexSelect(vertexloops);

  // restore old edges
  mesh->EdgeSelect(secondarysel);
}
void MNMesh_selectEdgeRingsAndLoops(MNMesh *mesh, int startid, BitArray &outsel)
{
  BitArray loopsel(mesh->ENum());
  outsel.SetSize(mesh->ENum());

  // ring
  outsel.Set(startid);
  mesh->SelectEdgeRing(outsel);

  // loop
  loopsel.Set(startid);
  mesh->SelectEdgeRing(loopsel);

  // combine both and set selection
  outsel |= loopsel;
}

void MNMesh_selectFaceLoops(MNMesh *mesh, int startid, BitArray &facesel)
{
  BitArray secondarysel(mesh->ENum());
  BitArray ringsel(mesh->ENum());
  facesel.SetSize(mesh->FNum());

  mesh->getEdgeSel(secondarysel);
  // convert to Edge
  MNFace *face = mesh->F(startid);
  for (int d = 0; d < face->deg; d++){
    ringsel.Set(face->edg[d]);
  }
  // ring
  mesh->SelectEdgeRing(ringsel);

  // back to face
  BitArrayEdgeToFace etf(mesh,&facesel);
  SM_BITARRAY_ENUMSET(ringsel,etf,BitArrayEdgeToFace);
  mesh->FaceSelect(facesel);

  // restor old edges
  mesh->EdgeSelect(secondarysel);
}