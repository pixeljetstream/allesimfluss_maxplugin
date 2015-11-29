/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "meshconnectivity.h"


//////////////////////////////////////////////////////////////////////////
// MNMeshConVert

size_t MNMeshConVertBuffer::MNMeshConVert_AdvanceConnected(int startindex, int endindex, Point3 &center, float maxdist, BitArray &allowed)
{
  MNMeshConVert* __restrict cur = &m_affected[startindex];
  MNMeshConVert* __restrict end = &m_affected[endindex];

  //for (int i = startindex; i < endindex; i++){
  //  int v = m_affected[i].vid;
  for (; cur < end; cur++){
    int v = cur->vid;
    Tab<int>  &connected = m_mesh->vedg[v];
    int numcon = connected.Count();
    for (int c = 0; c < numcon; c++){
      int nv = m_mesh->E(connected[c])->OtherVert(v);
      float dist;
      if (allowed[nv] && (dist=(m_mesh->P(nv)-center).Length()) < maxdist){
        MNMeshConVert_Push(nv,dist,allowed);
      }
    }
  }

  m_numaffected = m_lastcv-&m_affected[0];
  return m_numaffected;
}

size_t MNMeshConVertBuffer::MNMeshConVert_AdvanceConnected(int startindex, int endindex, BitArray &allowed,MNMeshConVertCallBack *cb)
{
  MNMeshConVert* __restrict cur = &m_affected[startindex];
  MNMeshConVert* __restrict end = &m_affected[endindex];

  //for (int i = startindex; i < endindex; i++){
  //  int v = m_affected[i].vid;
  for (; cur < end; cur++){
    int v = cur->vid;
    Tab<int>  &connected = m_mesh->vedg[v];
    int numcon = connected.Count();
    for (int c = 0; c < numcon; c++){
      int edgeid = connected[c];
      int nv = m_mesh->E(edgeid)->OtherVert(v);
      if (allowed[nv] && (!cb || cb->IsEdgeLegal(edgeid))){
        MNMeshConVert_Push(nv,0.0f,allowed);
      }
    }
  }

  m_numaffected = m_lastcv-&m_affected[0];
  return m_numaffected;
}


template<int distasweight, int dopos, int donormal>
class MNMeshConVertOpNP
{
public:
  __forceinline void Apply(MNMeshConVertIter it, const MNMeshConVertBuffer* mcon){
    int vid = it->vid;
    float wt = (distasweight ? it->dist : 1.0f);
    sum += wt;
    if (dopos) pos += m_mesh->P(vid)*wt;
    if (donormal) normal += m_mesh->GetVertexNormal(vid)*wt;
  }

  MNMeshConVertOpNP(MNMesh* mesh) :
    normal(0,0,0),pos(0,0,0),sum(0.0f),m_mesh(mesh) {
  }

  float sum;
  Point3 normal;
  Point3 pos;
  MNMesh* __restrict m_mesh;

};

Point3  MNMeshConVertBuffer::GetAffectedNormal(BOOL distasweight)
{
  if (distasweight){
    MNMeshConVertOpNP<1,0,1> op(m_mesh);
    ApplyOp<MNMeshConVertOpNP<1,0,1>>(op);
    return op.normal/op.sum;
  }
  else{
    MNMeshConVertOpNP<0,0,1> op(m_mesh);
    ApplyOp<MNMeshConVertOpNP<0,0,1>>(op);
    return op.normal/op.sum;
  }
}
Point3  MNMeshConVertBuffer::GetAffectedPos(BOOL distasweight)
{
  if (distasweight){
    MNMeshConVertOpNP<1,1,0> op(m_mesh);
    ApplyOp<MNMeshConVertOpNP<1,1,0>>(op);
    return op.pos/op.sum;
  }
  else{
    MNMeshConVertOpNP<0,1,0> op(m_mesh);
    ApplyOp<MNMeshConVertOpNP<0,1,0>>(op);
    return op.pos/op.sum;
  }
}
void  MNMeshConVertBuffer::GetAffectedNormalPos(Point3 &normal, Point3 &pos,BOOL distasweight)
{
  if (distasweight){
    MNMeshConVertOpNP<1,1,1> op(m_mesh);
    ApplyOp<MNMeshConVertOpNP<1,1,1>>(op);
    pos = op.pos/op.sum;
    normal = op.normal/op.sum;
  }
  else{
    MNMeshConVertOpNP<0,1,1> op(m_mesh);
    ApplyOp<MNMeshConVertOpNP<0,1,1>>(op);
    pos = op.pos/op.sum;
    normal = op.normal/op.sum;
  }
}

template <int touter>
class MNMeshConVertOpDistAsWeight
{
public:
  __forceinline void Apply(MNMeshConVertIter it, const MNMeshConVertBuffer* mcon){
    int vid = it->vid;
    float weight = 1.0f - min(max(0.0f,(it->dist-mindist))*invdist,1.0f);
    weight = SMOOTHERSTEP(weight);
    it->dist = weight;

    if (touter) weights[vid] = weight;
  }

  MNMeshConVertOpDistAsWeight(float mindist, float maxdist, float *weights) : mindist(mindist),weights(weights){
    invdist = 1.0f/max(0.01f,(maxdist-mindist));
  }

  float invdist;
  float mindist;
  float* __restrict weights;

};

void MNMeshConVertBuffer::AffectedDistWeights( float mindist, float maxdist )
{
  MNMeshConVertOpDistAsWeight<0>op(mindist,maxdist,NULL);

  ApplyOp<MNMeshConVertOpDistAsWeight<0>>(op);
}

void MNMeshConVertBuffer::AffectedDistWeightsArray( float mindist, float maxdist, float* outer/*=NULL*/ )
{
  // clear old
  float *weights = outer ? outer : &m_weights[0];
  memset(weights,0,sizeof(float)*m_mesh->VNum());

  MNMeshConVertOpDistAsWeight<1>op(mindist,maxdist,weights);
  ApplyOp<MNMeshConVertOpDistAsWeight<1>>(op);
}


void MNMeshConVertBuffer::BuildNormals()
{
  m_normals.resize(m_numaffected);

  MNMeshConVertIter it = m_affected.begin();
  MNMeshConVertIter itend = it+m_numaffected;


  MESHCONVERT_IT_MASK(m_masked)
    m_normals[it - m_affected.begin()] = m_mesh->GetVertexNormal(it->vid);
  MESHCONVERT_IT_NORM
    m_normals[it - m_affected.begin()] = m_mesh->GetVertexNormal(it->vid);
  MESHCONVERT_IT_END
}


void  MNMeshConVertBuffer::AffectedToBitArray(BitArray &bt, bool force)
{
  if (!m_masked && !force){
    bt = ~m_allowed;
    return;
  }

  MNMeshConVertIter it = m_affected.begin();
  MNMeshConVertIter itend = it+m_numaffected;

  bt.ClearAll();

  MESHCONVERT_IT_MASK(m_masked)
    bt.Set(it->vid);
  MESHCONVERT_IT_NORM
    bt.Set(it->vid);
  MESHCONVERT_IT_END
}


float* MNMeshConVertBuffer::GetWeights()
{
  return &m_weights[0];
}

void MNMeshConVertBuffer::BuildAffectedSoft(float maxdist, Point3 &center, bool allowmasked)
{
  m_allowed.SetAll();

  for (size_t i = 0; i < m_numaffected; i++){
    m_allowed.Clear(m_affected[i].vid);
  }

  size_t first = 0;
  size_t cfirst = 0;
  size_t last = m_numaffected;


  while(first < last){
    cfirst = first;
    first = last;
    last = MNMeshConVert_AdvanceConnected((int)cfirst,(int)last,center,maxdist,m_allowed);
  }

  if (allowmasked && !m_cursel.IsEmpty() && (m_firstselected || m_frozensel)){
    m_masked = TRUE;
    MNMeshConVert* __restrict curcv = &m_affected[0];
    for (; curcv < m_lastcv; curcv++){
      curcv->dist = m_cursel[curcv->vid] ? curcv->dist : -1.0f;
    }
  }
  else{
    m_masked = FALSE;
  }
}


void MNMeshConVertBuffer::BuildAffectedRigid(bool seedconnected,MNMeshConVertCallBack *cb)
{
  m_masked = FALSE;
  if (m_cursel.IsEmpty())
    return;

  if ((m_firstselected && !seedconnected) || m_frozensel){

    // remove first
    if (!m_firstselected){
      m_lastcv = &m_affected[0];
      m_numaffected = 0;
    }

    // fill in all selected
    int cnt = m_mesh->VNum();
    for (int i = 0; i < cnt; i++){
      if (m_cursel[i])
        MNMeshConVert_Push(i,0.0f,m_cursel);
    }
    m_numaffected = m_lastcv-&m_affected[0];
  }
  else if (m_firstselected && seedconnected){
    m_allowed = m_cursel;

    size_t first = 0;
    size_t cfirst = 0;
    size_t last = m_numaffected;


    while(first < last){
      cfirst = first;
      first = last;
      last = MNMeshConVert_AdvanceConnected((int)cfirst,(int)last,m_allowed,cb);
    }
  }
}

void  MNMeshConVertBuffer::SeedSingle(int subobj, int startid, float maxdist, Point3 &center, BOOL soft, BOOL frozensel, BOOL usesel, BOOL* overridesel)
{
  m_numaffected = 0;
  m_lastcv = &m_affected[0];

  m_masked = FALSE;
  m_firstselected = FALSE;
  m_frozensel = frozensel;
  

  switch(subobj){
  case SM_SO_VERTEX:
    {
      float dist = 0;
      if (!soft || (dist=(m_mesh->P(startid)-center).Length()) < maxdist ){
        m_mesh->getVertexSel(m_cursel);
        if (usesel) m_firstselected = overridesel ? *overridesel : m_cursel[startid];

        MNMeshConVert_Push(startid, dist, m_cursel, !soft);
        m_numaffected = 1;
      }
    }
    break;
  case SM_SO_EDGE:
    {
      MNEdge &edge = *m_mesh->E(startid);

      BitArray edges(m_mesh->nume);
      m_mesh->getEdgeSel(edges);

      if (usesel) m_firstselected = overridesel ? *overridesel : edges[startid];

      m_cursel.ClearAll();
      BitArrayEdgeToVertex bacb(m_mesh,m_cursel);
      TSMBitArray_EnumSet<BitArrayEdgeToVertex>(edges,bacb);

      for (int i = 0; i < 2; i++){
        float dist = 0;
        if (!soft || (dist=(m_mesh->P(edge[i])-center).Length()) < maxdist ){
          MNMeshConVert_Push(edge[i], dist, m_cursel, !soft);
          m_numaffected++;
        }
      }
    }
    break;
  case SM_SO_FACE:
    {
      MNFace *face = m_mesh->F(startid);
      BitArray faces(m_mesh->numf);
      m_mesh->getFaceSel(faces);

      if (usesel) m_firstselected = overridesel ? *overridesel : faces[startid];

      m_cursel.ClearAll();
      BitArrayFaceToVertex bacb(m_mesh,m_cursel);
      SM_BITARRAY_ENUMSET(faces,bacb,BitArrayFaceToVertex);

      
      for (int d = 0; d < face->deg; d++){
        float dist = 0;
        if (!soft || (dist=(m_mesh->P(face->vtx[d])-center).Length()) < maxdist ){
          MNMeshConVert_Push(face->vtx[d],dist, m_cursel, !soft);
          m_numaffected++;
        }
      }
    }
    break;
  }

}

void  MNMeshConVertBuffer::CheckMesh(MNMesh *mesh)
{
  m_mesh = mesh;
  if (m_vcnt >= mesh->VNum()) return;
  SetMesh(mesh);
}

void  MNMeshConVertBuffer::SetMesh(MNMesh* mesh)
{
  m_mesh = mesh;
  m_vcnt = mesh->VNum();

  m_weights.resize(m_vcnt);
  m_affected.resize(m_vcnt);

  m_cursel.SetSize(m_vcnt);
  m_allowed.SetSize(m_vcnt);
}

void MNMeshConVertBuffer::Clear()
{
  m_vcnt = 0;
  m_weights.clear();
  m_affected.clear();
  m_normals.clear();

  m_cursel.SetSize(1);
  m_allowed.SetSize(1);
}


//////////////////////////////////////////////////////////////////////////
// MeshEdge stuff


void MNMesh_advanceConnectedEdgeVerts(MNMesh *mesh, int edgeid, int vertid,
      BitArray &allowededges,Point4 *rangecheck, bool onlyopen, bool pushfront, int maxcnt,
      std::list<int> &vertlist,std::list<int> &edgelist)
{
  // v2 advancing = push_front
  int oldvert = vertid;
  int oldedgeid = edgeid;
  BOOL advanced = TRUE;
  maxcnt = maxcnt >= 0 ? maxcnt : 0xFFFF;
  int cnt = 0;
  while (advanced && cnt < maxcnt){
    advanced = FALSE;

    Tab<int> &edges = mesh->vedg[oldvert];
    int edgecnt = edges.Count();

    int canadvance = 0;
    int validedgeid = -1;
    for (int i = 0; i < edgecnt; i++){
      int nextedigeid = edges[i];
      if (nextedigeid == oldedgeid)
        continue;

      MNEdge *nextedge = mesh->E(nextedigeid);
      if (allowededges[nextedigeid] && (!onlyopen || nextedge->f2 < 0) &&
        (!rangecheck ||
          (rangecheck && // use 50% edge length
          (((mesh->P(nextedge->OtherVert(oldvert))+mesh->P(oldvert))*0.5f)
            - *((Point3*)rangecheck)).Length() < rangecheck->w)
        )){
        validedgeid = nextedigeid;
        canadvance++;
      }
    }
    // return if multiple possible/none exist
    if (canadvance != 1)
      return;

    MNEdge *validedge = mesh->E(validedgeid);
    oldvert = validedge->OtherVert(oldvert);
    oldedgeid = validedgeid;
    allowededges.Clear(validedgeid);

    if (pushfront){
      vertlist.push_front(oldvert);
      edgelist.push_front(oldedgeid);
    }
    else{
      vertlist.push_back(oldvert);
      edgelist.push_back(oldedgeid);
    }
    advanced = TRUE;
    cnt++;
  }
}

  // returns position in output list
int MNMesh_findConnectedEdgeVerts(MNMesh *mesh, int edgeid,BitArray &allowed,Tab<int> &edges,Tab<int> &verts, Point4 *rangecheck, bool reverse, bool onlyopen, int beforecnt, int aftercnt)
{
  MNEdge *edge = mesh->E(edgeid);
  std::list<int> vertlist;
  std::list<int> edgelist;

  int v2 = reverse ? edge->v1 : edge->v2;
  int v1 = reverse ? edge->v2 : edge->v1;

  vertlist.push_back(v2);
  vertlist.push_back(v1);

  edgelist.push_back(edgeid);

  allowed.Clear(edgeid);

  // v2 advanccing = push_front
  MNMesh_advanceConnectedEdgeVerts(mesh,edgeid,v2,allowed,rangecheck,onlyopen,true,beforecnt,vertlist,edgelist);

  // position in output list
  int pos = (int)(vertlist.size()-2);

  // v1 advancing = push_back
  MNMesh_advanceConnectedEdgeVerts(mesh,edgeid,v1,allowed,rangecheck,onlyopen,false,aftercnt,vertlist,edgelist);



  // output ordered  .. v2-v1 ...
  verts.SetCount((int)vertlist.size());
  int i = 0;
  std::list<int>::iterator it;
  for ( it=vertlist.begin() ; it != vertlist.end(); it++,i++ ){
    SM_ASSERT(i < verts.Count());
    verts[i] = *it;
  }

  edges.SetCount((int)edgelist.size());
  i = 0;
  for ( it=edgelist.begin() ; it != edgelist.end(); it++,i++ ){
    SM_ASSERT(i < edges.Count());
    edges[i] = *it;
  }

  return pos;
}


//////////////////////////////////////////////////////////////////////////
// MNMesh Vertex

BOOL MNMesh_areVertsConnected(MNMesh *mesh, int a, int b, int &viaedge)
{
  Tab<int> &edges = mesh->vedg[a];
  int ecnt = edges.Count();

  for (int i = 0; i < ecnt; i++){
    MNEdge *edge = mesh->E(edges[i]);
    if (edge->OtherVert(a) == b){
      viaedge = edges[i];
      return TRUE;
    }
  }
  return FALSE;
}