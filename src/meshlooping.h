/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_MESHLOOPING__H
#define __SKETCHMODELER_MESHLOOPING__H

/*
  MNMesh functions to generate loops/rings for
  all subobjs.

*/

#include "maxincludes.h"
#include "helpers.h"

//////////////////////////////////////////////////////////////////////////
// MNMeshLoop

typedef struct MNMeshLoopItem_s{
  Point3    pos;      // center/ref point local coords
  int     wave;     // distance steps to root
  int     id;       // subobj id
  float   distance;

  union{
    int         previndex;
    struct MNMeshLoopItem_s *prev;  // NULL for root
  };
  struct MNMeshLoopItem_s *next; // NULL for root and last (may not be set)
}MNMeshLoopItem;

typedef struct MNMeshLoopFront_s{
  int       previndex;
  int       previd;
  int       connector;
  int       contype;
  BOOL      crossed;

  int       nextid;
}MNMeshLoopFront;

//////////////////////////////////////////////////////////////////////////
// MNMeshLoopAdvancer

class MNMeshLoopAdvancer {
public:
  MNMesh  *m_mesh;

  MNMeshLoopAdvancer(MNMesh *mesh):m_mesh(mesh){}

  // returns TRUE on stop
  BOOL  Advance(MNMeshLoopFront* front, MNMeshLoopItem* itemfill);
  BOOL  Advance(MNMeshLoopFront* front, MNMeshLoopItem* itemfill, int itemindex, int wave, BitArray &finalsel);

  MNMeshLoopFront* FindDirectedFront(Tab<MNMeshLoopFront> &fronts, Point3 &spos, Point3 &dir, GraphicsWindow *gw);

  // returns
  virtual int   SetupFront(int startid, Tab<MNMeshLoopFront> &fronts) = 0;

  virtual Point3  GetPos(int id) = 0;
  virtual int   GetNextID(int connector, int contype, int curid) = 0;
  virtual int   GetNextConnector(int curconnector, int contype, int curid) = 0;

  virtual BOOL  AreConnected(int a, int b, int &con) = 0;
  virtual void  Select(int id, bool state=true) = 0;
  virtual void  GetSelected(BitArray &sel) = 0;
  virtual void  SetSelected(BitArray &sel) = 0;
  virtual int   GetCount() = 0;
};

class MNMeshLoopAdvancerVertex : public MNMeshLoopAdvancer{
public:
  MNMeshLoopAdvancerVertex(MNMesh *mesh):MNMeshLoopAdvancer(mesh){}

  // contype is ignored
  int    SetupFront(int startid, Tab<MNMeshLoopFront> &fronts);
  Point3 GetPos(int id);
  int    GetNextID(int connector, int contype, int curid);
  int    GetNextConnector(int curconnector, int contype, int curid);

  BOOL  AreConnected(int a, int b, int &con);
  void  Select(int id, bool state=true);
  void  GetSelected(BitArray &sel);
  void  SetSelected(BitArray &sel);
  int   GetCount();
};

class MNMeshLoopAdvancerEdge: public MNMeshLoopAdvancer{
private:
  int   m_disableFlag;
public:
  MNMeshLoopAdvancerEdge(MNMesh *mesh,int disableFlag=0): MNMeshLoopAdvancer(mesh), m_disableFlag(disableFlag){}


  // contype is loop=0 , ring=1
  int    SetupFront(int startid, Tab<MNMeshLoopFront> &fronts);
  Point3 GetPos(int id);
  int    GetNextID(int connector, int contype, int curid);
  int    GetNextConnector(int curconnector, int contype, int curid);

  BOOL  AreConnected(int a, int b, int &con);
  void  Select(int id, bool state=true);
  void  GetSelected(BitArray &sel);
  void  SetSelected(BitArray &sel);
  int   GetCount();
};

class MNMeshLoopAdvancerFace: public MNMeshLoopAdvancer{
public:
  MNMeshLoopAdvancerFace(MNMesh *mesh):MNMeshLoopAdvancer(mesh){}

  // contype is ignored
  int    SetupFront(int startid, Tab<MNMeshLoopFront> &fronts);
  Point3 GetPos(int id);
  int    GetNextID(int connector, int contype, int curid);
  int    GetNextConnector(int curconnector, int contype, int curid);

  BOOL  AreConnected(int a, int b, int &con);
  void  Select(int id, bool state=true);
  void  GetSelected(BitArray &sel);
  void  SetSelected(BitArray &sel);
  int   GetCount();
};

//////////////////////////////////////////////////////////////////////////
// MNMeshLoop

class MNMeshLoop
{
public:
  MNMeshLoop():m_numitems(0){};
  ~MNMeshLoop(){};

  // finalsel must have proper size
  void Build(MNMeshLoopAdvancer &adv, int startid, BitArray &finalsel);
  void LinkPrev();
  void Clear();

  Tab<MNMeshLoopItem> m_items;
  int         m_numitems;

};

//////////////////////////////////////////////////////////////////////////
// MNMesh

inline BOOL MNFace_containsEdge(MNFace *face, int eid)
{
  for (int d=0; d < face->deg; d++)
    if (face->edg[d]==eid)
      return TRUE;

  return FALSE;
}

inline BOOL MNMesh_areEdgesConnected(MNMesh *mesh, int aid, int bid, int &viavid)
{
  MNEdge* a = mesh->E(aid);
  MNEdge* b = mesh->E(bid);

  return (  (viavid=b->v1)<0 || // is false
    a->v1 == b->v1 ||
    a->v2 == b->v1 ||
        (viavid=b->v2)<0 || // is false
    a->v1 == b->v2 ||
    a->v2 == b->v2);

}

inline BOOL MNEdge_connected(MNEdge* a, int vert){
  return (a->v1 == vert || a->v2 == vert);
}

// finds the previous or next edge of an edge inside a face, connected via convert
// returns edgeid (-1 on fail) and stores edge* into conedge
int MNMesh_findConnectedFaceEdge(MNMesh *mesh, MNFace *face, int eid, int convert, MNEdge **conedge);

// using 3dsmax edge loop & ringing
/*
  loop: as long as one vert is connected to a quad, it propagates loop
  ring: if "+2 or -2" edge in face, is connected to a quad it works
  if both, then not
*/
void MNMesh_selectVertexLoops   (MNMesh *mesh, int startid, BitArray &outselect);
void MNMesh_selectEdgeRingsAndLoops (MNMesh *mesh, int startid, BitArray &outselect);
void MNMesh_selectFaceLoops     (MNMesh *mesh, int startid, BitArray &outselect);

// 3dsmax like behavior
int MNMesh_nextEdgeRingMax(MNMesh *mesh, int eid, int conface);
int MNMesh_nextEdgeLoopMax(MNMesh *mesh, int eid, int convert);

// simpler versions
int MNMesh_nextEdgeLoop   (MNMesh *mesh, int edge, int convert, BOOL aroundcorners=FALSE);
int MNMesh_nextEdgeRing   (MNMesh *mesh, int edge, int conface);

// same for both
int MNMesh_nextVertexLoop (MNMesh *mesh, int vert, int conedge);
int MNMesh_nextFaceLoop   (MNMesh *mesh, int face, int conedge);

#endif
