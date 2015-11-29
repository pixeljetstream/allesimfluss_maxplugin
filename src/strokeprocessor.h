/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_STROKEPROCESSOR__H
#define __SKETCHMODELER_STROKEPROCESSOR__H

#include "maxincludes.h"
#include "helpers.h"
#include "workplane.h"


class MNMeshConVertBuffer;

#define SM_STRIPFACE_FLAG (MN_USER<<1)

//////////////////////////////////////////////////////////////////////////
// StrokePoint

typedef struct WeldPair_s{
  int ovid; // host mesh id
  int avid; // pre-attached vertex id
  int nextedge; // edge id of "next" edge one can weld with
}WeldPair;

enum  StrokeFlags_e{
  SM_STROKEFLAG_NONE    = 0,
  SM_STROKEFLAG_LOCK    = 1<<0,
  SM_STROKEFLAG_DEAD    = 1<<1,
  SM_STROKEFLAG_NORELAX = 1<<2,
};

typedef struct StrokePoint_s{
  DWORD     flag;
  Point3    pos;     // used point (after relax)
  Point3    opos;    // original point
  float     size;
  float     onpath;

  int       startvid; // start vertex id + m_numdeltas
  int       startfid; // start face id
  int       dynweld[2];

  Matrix3   deltamat;
}StrokePoint;

typedef std::vector<StrokePoint>  StrokePointArray;
typedef std::vector<StrokePoint>::iterator      StrokePointIter;
typedef std::vector<StrokePoint>::const_iterator  StrokePointIterConst;


//////////////////////////////////////////////////////////////////////////
// StrokeProcessor
class StrokeProcessor
{
public:

  enum StrokeMode {
    STR_NONE,
    STR_FREE,
    STR_EDGE,
    STR_FACE,
  };

  enum SelectionMode {
    SEL_NOCHANGE,
    SEL_ADD,
    SEL_ONLY,
  };

  enum RailMode {
    RAIL_NONE,
    RAIL_1D,
    RAIL_2D,
  };

  enum WeldMode {
    WELD_SNAPOUTERVERTS,
    WELD_SNAPALLVERTS,
  };

  enum DeltaStrideMode {
    DELTASIZE_MAX,
    DELTASIZE_MIN,
    DELTASIZE_AVG,
  };

  StrokeProcessor();
  ~StrokeProcessor(){}

  void  PrepStart(MNMesh *ref, MNMesh *out, WorkPlane *plane, Matrix3 objtm);
  void  Init(MNMeshConVertBuffer *convert) {m_convert = convert;}

    // new hits coming in (from pen)
  void  CheckHit(Point3 &hit, float brushsize,ViewExp *vpt);

  void  ChangeOutput(MNMesh *out);

    // edgeids only allowed (>=0) when open
    // initializes deltas
  void  Start(Point3 &hit, int startedgeid, int startfaceid, float size, const Matrix3* clustermatrix=NULL);
  void  Refine(StrokeProcessor& strokeproc);

  // end
  void  End(int endedgeid, int endfaceid, const Point3 &endpt);

  void  ApplyChanges(int subobj, SelectionMode selmode);
  void  DiscardChanges();

  void  ResetToStart();
  void  Replay(const StrokeProcessor& strokeproc, bool flipSide, bool scale);
  BOOL  SaveReplay( Tab<float> &replay ) const;
  BOOL  LoadReplay( const Tab<float> &replay );
  BOOL  ExtractReplay(int seededge, int seedface, MNMesh *mesh, WorkPlane *plane, const Matrix3& objtm);

  BOOL  CanReplay() const{
    return m_lastmode != STR_NONE && m_strokepoints.size() > 1;
  }

    // tweakables
  void  SetStrideMulti(float val) {m_stridemulti = max(0.1f,val);}
  float GetStrideMulti() const { return m_stridemulti;}
  void  SetStrideMode(DeltaStrideMode mode) { m_deltastridemode = mode; }
  DeltaStrideMode  GetStrideMode() const { return m_deltastridemode; }
  void  SetRailMode(RailMode mode) {m_railmode = mode;}
  RailMode GetRailMode() const { return m_railmode;}
  void  SetWeldFactor(float val) {m_dynweldfactor = val;}
  float GetWeldFactor() const { return m_dynweldfactor;}
  void  SetWeldMode(WeldMode mode) {m_dynweldmode = mode;}
  WeldMode GetWeldMode() const { return m_dynweldmode;}
  void  SetRelaxFactor(float val) {m_relaxfactor = val;}
  float GetRelaxFactor() const { return m_relaxfactor;}
  void  SetRelaxRuns(int val) {m_relaxruns = max(0,val);}
  int   GetRelaxRuns() const { return m_relaxruns;}
  void  SetRotateFree(float val) {m_rotfree = val;}
  float GetRotateFree() const { return m_rotfree;}
  void  SetRotateFactor(float val) {m_rotfactor = val;}
  float GetRotateFactor() const { return m_rotfactor;}
  void  SetEdgeCorrectInplane(BOOL state) { m_edgecorrinplane = state;}
  BOOL  GetEdgeCorrectInplane() const { return m_edgecorrinplane; }
  void  SetEdgeCorrectThreshold(float state) { m_edgecorrthresh = state;}
  float GetEdgeCorrectThreshold() const { return m_edgecorrthresh;}
  void  SetRemoveThreshold(float state) { m_removethresh = state;}
  float GetRemoveThreshold() const { return m_removethresh;}
  void  SetSmoothing(BOOL state) { m_smooth = state;}
  BOOL  GetSmoothing() const { return m_smooth; }
  void  SetSmoothThreshold(float state) { m_smooththresh = state;}
  float GetSmoothThreshold() const { return m_smooththresh;}
  void  SetFreeDeltas(const Tab<Point3> &freedeltas){m_freedeltas = freedeltas; }
  void  SetExplicitStride(BOOL state) { m_explicitstride = state;}
  BOOL  GetExplicitStride() const { return m_explicitstride;}
  void  SetConnectInterpolation(const SMInterpolation& interpolate) {
    m_interpolator = interpolate;
  }
  SMInterpolation&  GetConnectInterpolation(SMInterpolation& interpolate) const {
    interpolate = m_interpolator;
    return interpolate;
  }
  void  SetOpenInterpolation(BOOL state) { m_openInterpolation = state; }
  BOOL  GetOpenInterpolation() {return m_openInterpolation; }
  void  SetOpenOffset(float state) { m_endoffset = state; }
  float GetOpenOffset() {return m_endoffset; }

    // misc
  BOOL    HasPoints(int num=2)  { return m_lastpoint >= num-1;}
  Point3  GetLastFixedPoint()   { return m_strokepoints[m_lastpoint-1].pos;}
  Point3  GetFirstFixedPoint()  { return m_strokepoints[0].pos;}

  int   GetStartEdges(int seededge, int startface, const Point3 &brushpt,float brushsize, Tab<int> &edges,Tab<int> &edgeverts,MNMesh *overridemesh=NULL);
  void  GetFaceAttribs(const Tab<int> &edges, int startface, Tab<FaceAttrib> &fattribs, DWORD &sgs);
  BOOL  GetEndEdges(int seededge,Tab<int> &edges,Tab<int> &edgeverts, Point3 &anchor);
  BOOL  OverlapsWithStart(int seededge);
  

  const StrokePointArray& GetStrokePointArray(int &lastpoint) const {
    lastpoint = m_lastpoint;
    return m_strokepoints;
  }

  FORCEINLINE int   getNumDeltas() {
    return m_numdeltas;
  }

    // cleanup function externally called only for aborting
  void  Clear();
private:

  //////////////////////////////////////////////////////////////////////////
  // Creation/Modification
  void  ConnectPoints(int self, int prev);
  void  AddPoint(const Point3 &hit, const Matrix3 &mat, float size);
  StrokePoint& AddPointRaw();
  void  SetPoint(int pt, const Point3 &hit, float size);


  void  RemoveLastTooClose(float sizemulti);
  void  RemoveLastPointFaces();
  void  RemoveLastPoint();
  void  RemoveDeadPoints();

  //////////////////////////////////////////////////////////////////////////
  // Updating
  void  DeltaMatrix(Matrix3 &mat, const Point3 &pos, const Point3 &tonext, int pt, float size);
  Point3* CollAlignNormal(const Point3& sp, Point3 &normal);
  Point3  CollCorrectPoint(const Point3& sp);

  void  UpdateVertices(const StrokePoint &sp, const Matrix3 &mat);
  void  UpdatePoint(int pt);
  void  UpdateAll(int from, int to);


  //////////////////////////////////////////////////////////////////////////
  // Immediate checks
  BOOL  OverlapCheck();


    // interpolate and add based on stride
  int   AddInterPoints(Point3 &hit, float size,Point3 &tohit, float dist, float stride, int max=1);
  void  RelaxPoints();
  BOOL  DynamicWeld(int from, int to, WeldMode mode);
  void  DynamicWeldFinalize();

  //////////////////////////////////////////////////////////////////////////
  // Starting

  void  StartFromFree( Point3 & hit, float size );
  int   CorrectStartEdges(MNMesh* __restrict mesh, int seededgepos, Tab<int> &edges, Tab<int> &edgeverts);
    // m_deltamats are set
  void  InitFromEdgeOrFace(Tab<int> &edgeverts, const Point3 &hit, int startedge, int startface, float size, const Matrix3 &wpmat, const Matrix3 &wpmatinv);
  void  StartFromEdge(Point3 &hit, int startedge, float size, const Matrix3 &wpmat, const Matrix3 &wpmatinv);
    // extrude
  void  StartFromFace(Point3 &hit, int startedge, int startface, float size, const Matrix3 &wpmat, const Matrix3 &wpmatinv,const Matrix3* clustermatrix);

  //////////////////////////////////////////////////////////////////////////
  // Ending

  void  EndAddPoints();
  void  EndFixTriangles(Tab<int> &endverts,int &firsttri, int &lasttri);
  void  EndLerpStrokepoints(const Point3 &endpt,const Matrix3& endmat, bool reproject=true);
  void  EndLerpDeltas(const Point3 &endpt, const Matrix3 &endmat, const Point3 &wnormal);
  BOOL  EndAtEdge(int endedgeid, const Point3 &endpt);
  void  RelaxSlideVertices(int from, int to, int runs, float weight);

  void  EndFace();
  void  EndEdge();

  //////////////////////////////////////////////////////////////////////////
  // Internal Ops
  void  InitDeltaSize();
  void  UpdateDeltas(const Matrix3 &tm);
  Matrix3 GetEdgeMat(const Point3 &target, const Point3 &pos, const Point3 &normal);
  float GetStride(float brushsize);
  float GetSize(float brushsize);
  Point3 ReprojectPoint(const Point3 &pt, float offset);
  

  // basic outer interfaces
  WorkPlane               *m_wplane;
  MNMesh                  *m_refmesh;
  MNMesh                  *m_outmesh;
  MNMeshConVertBuffer     *m_convert;
  BOOL                    m_lastconstrained;
  StrokeMode              m_mode;
  StrokeMode              m_lastmode;
  WorkPlane               m_lastwplane;
  DeltaStrideMode         m_deltastridemode;
  int                     m_lastmeshv;
  int                     m_lastmeshe;
  int                     m_lastmeshf;

  RailMode                m_railmode;
  bool              m_ended;
  int               m_endface;
  int               m_endedge;

  // refmesh flags for start-end connection
  BitArray          m_unprocedges;  // only for multi stroke
  BitArray          m_openendedges;

  BitArray          m_faces;      // cap faces from extrusion
  BitArray          m_faceverts;  // cap face vertices...

  // face-start related
  int             m_capstartfid;  // face/vertex ids in outmesh
  int             m_numcapfaces;
  int             m_capstartvid;
  int             m_numcapverts;
  Tab<Point3>     m_capdeltas;
  Tab<Point3>     m_capdeltasorig;

  int                   m_startface;  // reference start face
  Tab<FaceAttrib>       m_fattribs;   // reference face attribs
  std::vector<int>      m_delfaces;   // face ids in refmesh of to-delete cap

  // edge-start related
  int             m_startedge;  // reference start edge
  Tab<int>        m_startedges;
  float           m_startfracc;
  Point3          m_startrelative;
  BOOL            m_closedring;
  int             m_refedgepos; // in multiedge this tells us in which position
                                // the reference edge was

  // paint moderelated
  BOOL            m_dynweld;       // allow hug-weld
  DWORD           m_rail;
  int             m_deltadelay; // generates delta matrix with that strokepoint
  DWORD           m_invalidsgs;

  Matrix3           m_objtm;    // object-space of painted node
  Matrix3           m_objtminv;
  Matrix3           m_wp2obj; // workplane to obj
  Matrix3           m_obj2wp;

  Matrix3           m_startmat; // face/edge matrix in obj
  Matrix3           m_endmat;
  Matrix3           m_edgedeltamat;
  StrokePointArray  m_strokepoints;
  StrokePointArray  m_preendpoints;
  int               m_lastpoint;
  float             m_startstride;

  // strut vertices
  Tab<Point3>       m_deltas;     // offset vertices of struts
  Tab<Point3>       m_deltasorig;
  Point3            m_deltactr;
  Point3            m_deltactrorig;
  int               m_numdeltas;
  int               m_lastdelta;
  Tab<int>          m_connectids[2];// paint updates outmesh vertexids for connection quads
  float             m_deltasize;

  // weld-hug/extrusion
  std::vector<WeldPair>   m_wpairs;   // weldpairs of refmesh-outmesh vertices
  int                     m_lastweld;
  std::vector<WeldPair>   m_dynwpairs[2];
  int                     m_lastdynamic[2];

  

  // tweakables
  float           m_rotfactor;
  float           m_rotfactorold;
  float           m_rotfree;
  float           m_dynweldfactor;
  WeldMode        m_dynweldmode;
  float           m_relaxfactor;
  int             m_relaxruns;
  float           m_stridemulti;
  BOOL            m_edgecorrinplane;
  float           m_edgecorrthresh;
  float           m_removethresh;
  BOOL            m_smooth;
  float           m_smooththresh;
  Tab<Point3>     m_freedeltas;
  BOOL            m_explicitstride;
  BOOL            m_stretchconstrain;
  SMInterpolation m_interpolator;
  BOOL            m_openInterpolation;
  float           m_endoffset;
};

#endif
