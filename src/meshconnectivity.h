/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_MESHCONNECTIVITY__H
#define __SKETCHMODELER_MESHCONNECTIVITY__H

/*
MNMesh functions to find connected geometry (vertices)

*/

#include "maxincludes.h"
#include "helpers.h"

//////////////////////////////////////////////////////////////////////////
// MNMeshConVert

#define MESHCONVERT_IT_MASK(masked)       \
  if (masked)                 \
    for ( ; it < itend; it++ ){       \
      if (it->dist < 0.0f) continue;

#define MESHCONVERT_IT_NORM           \
    }                   \
  else                    \
    for ( ; it < itend; it++ ){

#define MESHCONVERT_IT_END    \
    }


struct MNMeshConVert{
  int     vid;
  float   dist;
  int     splinepos;
};

typedef std::vector<MNMeshConVert>::iterator  MNMeshConVertIter;

class MNMeshConVertCallBack {
public:
  virtual BOOL IsEdgeLegal(int index) = 0;
};

class MNMeshConVertBuffer
{
public:
  MNMeshConVertBuffer() : m_masked(0),m_numaffected(0),m_mesh(NULL),m_vcnt(0) {}
  ~MNMeshConVertBuffer(){}


  inline void GetIterators(MNMeshConVertIter &itstart, MNMeshConVertIter &itend)
  {
    itstart = m_affected.begin();
    itend = itstart+m_numaffected;
  }

  void  SeedSingle(int subobj, int startid, float maxdist, Point3 &center, BOOL soft, BOOL frozensel, BOOL usesel=FALSE, BOOL* overridesel=NULL);

    // performs ranged search
    // masking means that only selected are allowed (seed selected/frozensel)
  void  BuildAffectedSoft(float maxdist, Point3 &center, bool allowmasked=true);
    // uses seed or all selected (seed selected / frozensel)
    // if selelement is true, then only connected to seed are allowed
  void  BuildAffectedRigid(bool seedconnected=false, MNMeshConVertCallBack *cb=NULL);

    // converts meshconverts dist to weight (dist is overwritten)
  void AffectedDistWeights(float mindist, float maxdist);
    // computes weight into Weights float array(1 per vertex)
  void AffectedDistWeightsArray(float mindist, float maxdist, float* outer=NULL);

  void  AffectedToBitArray(BitArray &bt, bool force=false);

  template <class op>
  __forceinline void ApplyOp(op &opvar)
  {
    MNMeshConVertIter it = m_affected.begin();
    MNMeshConVertIter itend = it+m_numaffected;

    MESHCONVERT_IT_MASK(m_masked)
      opvar.Apply(it,this);
    MESHCONVERT_IT_NORM
      opvar.Apply(it,this);
    MESHCONVERT_IT_END
  }

  Point3  GetAffectedNormal(BOOL distasweight);
  Point3  GetAffectedPos(BOOL distasweight);
  void    GetAffectedNormalPos(Point3 &normal, Point3 &pos,BOOL distasweight);
  size_t  GetNumAffected()
  {
    return m_numaffected;
  }

  void  SetMesh(MNMesh* mesh);
  void    CheckMesh(MNMesh *mesh);
  void  Clear();

  void  BuildNormals();
  __forceinline const Point3& MNMeshConVertBuffer::GetNormal( MNMeshConVertIter iter ) const
  {
    return m_normals[iter - m_affected.begin()];
  }

  BOOL    GetSeedSelected() {return m_firstselected; }
  BOOL    GetMasked() {return m_masked;}
  float*    GetWeights();
  BitArray& GetUntouched() {return m_allowed;}

private:
  std::vector<MNMeshConVert>  m_affected;
  std::vector<float>      m_weights;
  std::vector<Point3>     m_normals;

  MNMeshConVert*        m_lastcv;
  size_t            m_numaffected;

  inline void MNMeshConVert_Push(int id, float dist, BitArray &allowed){
    MNMeshConVert inf;
    inf.vid = id;
    inf.dist = dist;
    allowed.Clear(id);

    *m_lastcv = inf;
    m_lastcv++;
  }
  inline void MNMeshConVert_Push(int id, float dist, BitArray &allowed, BOOL clear){
    MNMeshConVert inf;
    inf.vid = id;
    inf.dist = dist;
    if (clear)
      allowed.Clear(id);

    //m_affected[m_numaffected++] = inf;
    *m_lastcv = inf;
    m_lastcv++;
  }
  size_t MNMeshConVert_AdvanceConnected(int startindex, int endindex, Point3 &center, float maxdist, BitArray &allowed);
  size_t MNMeshConVert_AdvanceConnected(int startindex, int endindex, BitArray &allowed,MNMeshConVertCallBack *cb);

  MNMesh* __restrict  m_mesh;
  BOOL        m_masked;

  BOOL        m_firstselected;
  BOOL        m_frozensel;

  int         m_vcnt;
  BitArray      m_cursel;
  BitArray      m_allowed;
};



//////////////////////////////////////////////////////////////////////////
// MNMesh Vertex

BOOL MNMesh_areVertsConnected(MNMesh *mesh, int a, int b, int &viaedge);

//////////////////////////////////////////////////////////////////////////
// MNMesh Edge stuff

  // returns position of edge in series
  // negative cnts mean no limit
int MNMesh_findConnectedEdgeVerts(MNMesh *mesh, int edgeid,BitArray &allowed,
                  Tab<int> &edges,Tab<int> &verts,
                  Point4 *rangecheck=NULL,bool reverse=false, bool onlyopen=true,
                  int beforecnt=-1, int aftercnt=-1);


#endif
