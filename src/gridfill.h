/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_GRIDFILL__H
#define __SKETCHMODELER_GRIDFILL__H

#include "maxincludes.h"
#include "helpers.h"


#if defined(SM_SUPPORT_FILL)

//////////////////////////////////////////////////////////////////////////
// GridFillConfig

struct GridCapConfig{
  IPoint2 pole;
};

//////////////////////////////////////////////////////////////////////////
// GridInterpolator


class GridInterpolator{
public:
  GridInterpolator() : m_precise(1000), m_weight(0),m_tangentweight(1,1,1,1),m_inner(TRUE) {}
  virtual ~GridInterpolator() {}

  float   m_weight;
  Point4  m_tangentweight;
  int     m_precise;
  int     m_inner;
  Point3 Run( const Point2 &fracc, const Point3 & pfromoutX, const Point3 & pfromX, const Point3 & ptoX, const Point3 & ptooutX, const Point3 & pfromoutY, const Point3 & pfromY, const Point3 & ptoY, const Point3 & ptooutY );

  virtual void   Init() { }
  virtual Point3 Run(float fracc, float dist,
    const Point3 &prev, const Point3 &a, const Point3 &b, const Point3 &next, const Point2& tanweight)  = 0;

};

//////////////////////////////////////////////////////////////////////////
// GridLayout


class GridLayout{
public:

  struct Side {
    IPoint2   coord;
    IPoint2   dir;
    IPoint2   sign;

    int       size;
    int       startidx;
    BOOL      startspecial;
    int       outstart;
  };

  struct BorderPoint{
    int     vid;
    float   special;
    IPoint2 coord;
    IPoint2 sign;
    Point2  fcoord;
    float   sidelength;
    float   fracc;
    Point3  inner;
    Point3  outer[2];   // opposite edge vertex horizontal/vertical
    Point3  tangent[2];
  };

  GridLayout() {}
  ~GridLayout() {}

  BOOL  Init(MNMesh *mesh, Tab<int> *borderedges);
  void  BuildPoints();
  void  BuildAndUseInnerTangents();

  MNMesh*           m_mesh;
  IPoint2           m_dim;
  int               m_fillcnt;
  int               m_sidecnt;
  int               m_cornercnt;
  int               m_useinner;
  std::vector<BorderPoint>   m_gridouterpts;

  void InterpolateSide(int sideIdx, float fracc, Point3 &inner, Point3 &outer) const;

  inline const BorderPoint& GetBorderPoint(int idx) const{
    return m_gridouterpts[(idx+m_gridouterpts.size()) % m_gridouterpts.size()];
  }
  inline BorderPoint& GetBorderPoint(int idx){
    return m_gridouterpts[(idx+m_gridouterpts.size()) % m_gridouterpts.size()];
  }
  inline const Side& GetSide(int idx) const { 
    return m_sides[idx];
  }
  inline Side& GetSide(int idx){ 
    return m_sides[idx];
  }
  inline int  GetEdge(int idx) const {
    return m_borderedges[(idx+m_edgecnt+1)%m_edgecnt];
  }
  inline int  GetVertex(int idx) const {
    return m_borderverts[(idx+m_edgecnt)%m_edgecnt];
  }

private:

  int   GetCornerVertex(const BitArray &edges, const BitArray &verts, const BitArray &faces,
              BitArray &cornerverts, BitArray &cornerspecial);
  int   TryFourCorners(const BitArray &edges, const BitArray &verts, const BitArray &faces,
              BitArray &cornerverts, BitArray &cornerspecial);
  BOOL  BuildGridSides(int seedcorner, const BitArray &edges,
              const BitArray &cornerverts, const BitArray &cornerspecial);
  void GetSideMatrix(GridLayout::Side &gside, int pt, Matrix3& orientation);
  std::vector<Side>       m_sides;
  Tab<int>                m_borderedges;
  Tab<int>                m_borderverts;
  int                     m_edgecnt;
  
};


//////////////////////////////////////////////////////////////////////////
// GridCap


class GridCap {
public :

  enum TPointType {
    TPT_CORNER0,
    TPT_CORNER1,
    TPT_CORNER2,
    TPT_CORNER3,
    TPT_GP,
    TPT_G1,
    TPT_GN,
    TPT_G2,
    TPT_E0,
    TPT_E1,
    TPT_E2,
    TPT_E3,
    TPT_E4,
    TPT_E5,
    TPT_E6,
    TPT_E7,
    TPTS,
  };

  // IMPORTANT: tab doesn't call constructor

  struct TSide {
    int       layoutSide;   // init -1
    int       size;
  };

  struct TVert {
    int         vid;
    Point2      coord;
  };

  struct TPoint {
    TVert     vert;
    int       onside;     // init -1
    int       sideidx;

    bool TPoint::operator==(const TPoint &other) const {
      return vert.vid == other.vert.vid;
    }

    bool TPoint::operator!=(const TPoint &other) const {
      return vert.vid != other.vert.vid;
    }

    bool isOnSide() const {
      return onside >= 0;
    }
  };

  struct TEdge {
    bool        outer;
    int         size;
    TPoint*     from;
    TPoint*     to;
    Tab<TVert>  verts;
    Tab<float>  fraccs;

    inline float GetFracc(int i, bool reverse) const 
    {
      int cnt = size-1;
      float f = fraccs[reverse ? cnt-i-1 : i];
      return reverse ? 1.0f-f : f;
    }

    inline void SetFracc(int i, bool reverse, float f) 
    {
      int cnt = size-1;
      fraccs[reverse ? cnt-i-1 : i] = reverse ? 1.0f-f : f;
    }

    inline const TVert& GetVert(int i, bool reverse) const
    {
      return verts[ reverse ? size-i : i];
    }
  };

  struct TSector {
    int         numFaces;
    int         numEdges;
    TEdge*      edges[4];
    bool        reversed[4];
  };

  enum FillType {
    FILL_EVEN,
    FILL_UNEVEN,
    FILL_EQUAL,
    FILL_SPLIT,
    FILL_INVALID,
  };

  struct TConfig {
    FillType    type;
    int         alpha;
    int         beta;
    int         summed;
    int         fractions[2][2];
    int         sectorsizes[10];

    void Init(int sizes[4], const GridCapConfig &cfg);
  };

  struct Topo {
    TConfig       cfg;
    Tab<TSide>    sides;
    Tab<TPoint>   points;
    Tab<TEdge>    edges;
    Tab<TSector>  sector;

    int           numEdges;
    int           numSectors;

    Topo() : numEdges(0), numSectors(0)
    {}

    inline TEdge*    GetEdge(int a, int b){
      for (int i = 0; i < numEdges; i++){
        if ((*edges[i].from == points[a] || *edges[i].from == points[b]) &&
            (*edges[i].to   == points[a] || *edges[i].to   == points[b])){
          return &edges[i];
        }
      }
      return NULL;
    }
  };

  static bool ValidLayout(const GridLayout &gridlayout);
  static bool ValidLayout(const std::vector<int> &inputsides);
  static void GetSidesSizes(const std::vector<int>&inputsides, int sides[2], int sizes[4]);

  void    Run( GridLayout *gridlayout, GridInterpolator* gip, const GridCapConfig &cfg, Tab<int> *newFaces);

  GridCap()   {}
  ~GridCap()  {}

private:

  MNMesh* __restrict                  m_mesh;
  GridLayout* __restrict              m_layout;
  GridInterpolator* __restrict        m_gip;
  bool                                m_layoutReverse;

  void    InitSides   (Topo &topo, const std::vector<int> &inputsides, const GridCapConfig &cfg);
  BOOL    MustSplit   (Topo &topo);
  void    Fill        (Topo &topo);
  void    Split       (const Topo &topo, Topo &upper, Topo &lower);
  void    Merge       (Topo &topo, const Topo &upper, const Topo &lower);

  void    MakeSidePoint (Topo &topo, TPoint &tp, int side, int idx);
  void    AddSector     (Topo &topo, int a, int b, int c, int d, int size);
  void    AddEdge       (Topo &topo, int a, int b, int size);
  
  void    CreateEdgeVertices(Topo &topo, TEdge &edge);
  void    CreateSectorMesh  (Topo &topo, TSector &sector, Tab<int> *newFaces);
  void    CreateMesh(Topo &topo, Tab<int> *newFaces);

  Point3  Interpolate     (Topo &topo, const Point2& coord);
  void    CreateInnerPoint(Topo &topo, TPoint &tp, Point2 points[4], int weights[4]);
  
  
};


//////////////////////////////////////////////////////////////////////////
// GridFill


class GridFill {
public :

  typedef GridLayout::BorderPoint  OuterPoint;

  struct InnerPoint{
    int          vid;

    OuterPoint  *from;
    OuterPoint  *to;
  };

  struct Point{
    union{
      GridLayout::BorderPoint  *outer;
      int         inneridx;
    };
    BOOL          isinner;
  };

  GridFill() : m_grid(NULL) {}
  ~GridFill() {}

  void  Run(GridLayout *gridlayout, GridInterpolator *gip);

private:
  inline int grididx(int x, int y){ return y * m_dim.x + x;}
  inline int grididx(IPoint2 coord){ return coord.y * m_dim.x + coord.x;}

  void  BuildGridOuterPoints();

  void  CreateMeshVertex(InnerPoint &gin, OuterPoint *from, OuterPoint *to);

  void  CreateMeshVertexLine(OuterPoint *from, OuterPoint *to, int startidx, int curidx, int stride);
  void  CreateInnerPointLine(OuterPoint *from, OuterPoint *to, int startidx, int curidx, int stride);
  void  CreateGridMesh();

  void  Clear();

  MNMesh* __restrict      m_mesh;
  GridLayout* __restrict  m_layout;
  IPoint2                 m_dim;


  Point*                  m_grid;
  std::vector<InnerPoint> m_gridinnerpts;
  int                     m_numinner;
  int                     m_startvid;

  GridInterpolator* __restrict   m_gip;

};

//////////////////////////////////////////////////////////////////////////
// Outer Function

BOOL  PolyObject_GridFill(PolyObject *pobj, const SMInterpolation& interpolation, const GridCapConfig &cfg, int edge=-1);


#endif

#endif
