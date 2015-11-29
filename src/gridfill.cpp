/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "gridfill.h"
#include "helpers.h"
#include "meshconnectivity.h"
#include "meshlooping.h"
#include "meshrelax.h"
#include "restores.h"
#include <algorithm>

#if defined(SM_SUPPORT_FILL)

//////////////////////////////////////////////////////////////////////////
// GridInterpolator

__forceinline Point3 GridInterpolator::Run( const Point2 &fracc, const Point3 & pfromoutX, const Point3 & pfromX, const Point3 & ptoX, const Point3 & ptooutX, const Point3 & pfromoutY, const Point3 & pfromY, const Point3 & ptoY, const Point3 & ptooutY ) 
{

  float distX = Distance(pfromX,ptoX);
  float distY = Distance(pfromY,ptoY);

  Point3 posX = Run(fracc.x, distX, pfromoutX, pfromX, ptoX, ptooutX,Point2(m_tangentweight.x,m_tangentweight.y));

  Point3 posY = Run(fracc.y, distY, pfromoutY, pfromY, ptoY, ptooutY,Point2(m_tangentweight.z,m_tangentweight.w));

  float mix;

  if (m_weight != 0){
    mix = m_weight * 0.5 + 0.5;
  }
  else{
    distX = min(Distance(posX,ptoX),Distance(pfromX,posX));
    distY = min(Distance(posY,ptoY),Distance(pfromY,posY));
    float divsumdist = 1.0f/(distX+distY);
    mix = distX * divsumdist;
  }

  Point3 pos  = LERP(posX,posY,mix);
  return pos;
}

class GridInterpolatorLinear : public GridInterpolator{
public:
  static const SMInterpolatorType TYPE = SM_INTERPOLATOR_LINEAR;

  Point3 Run(float fracc, float dist,
    const Point3 &prev, const Point3 &a, const Point3 &b, const Point3 &next, const Point2& tanweight) 
  {
    return LERP(a,b,fracc);
  }
};

class GridInterpolatorCatmullRom : public GridInterpolator{
public:
  static const SMInterpolatorType TYPE = SM_INTERPOLATOR_CATMULLROM;

  Point3 Run(float fracc, float dist,
    const Point3 &prev, const Point3 &a, const Point3 &b, const Point3 &next, const Point2& tanweight) 
  {
    Point3 cprev = -(prev-a).Normalize();
    Point3 cnext =  (next-b).Normalize();

    SetupCatmullRom(a,b,cprev,cnext,tanweight);
    return CATMULLROM(cprev,a,b,cnext,fracc);
  }
};

class GridInterpolatorBezier : public GridInterpolator{
public:
  static const SMInterpolatorType TYPE = SM_INTERPOLATOR_BEZIER;

  Point3 Run(float fracc, float dist,
    const Point3 &prev, const Point3 &a, const Point3 &b, const Point3 &next, const Point2& tanweight) 
  {
    Point3 cprev = -(prev-a).Normalize();
    Point3 cnext =  (next-b).Normalize();

    SetupBezier(a,b,cprev,cnext,tanweight);

    m_precise = 10000;
    if (!m_precise){
      return BEZIER(a,cprev,cnext,b,fracc);
    }
    else{
      int steps = m_precise;
      const int NUMBETWEEN = 11;
      const float between = 1.0f/float(NUMBETWEEN+1);
      float lengths[NUMBETWEEN];

      // fracc was distance based, need to resample
      float curvedist = 0.0f;
      Point3 last = a;
      int subset = 0;
      for (int i = 1; i < steps; i++){
        float f = float(i)/(steps-1);
        Point3 cur = BEZIER(a,cprev,cnext,b,f);
        curvedist += Distance(cur,last);
        if (subset < NUMBETWEEN &&  f >= float(subset+1) * between){
          lengths[subset++] = curvedist;
        }
        last = cur;
      }


      curvedist = -curvedist * fracc;
      float startfracc = 0;
      for (int i = NUMBETWEEN-1; i >= 0; i--){
        if (curvedist + lengths[i] < 0.0){
          startfracc = between * float(i+1);
          curvedist += lengths[i];
          break;
        }
      }
      last = BEZIER(a,cprev,cnext,b,startfracc);
      steps /= (NUMBETWEEN+1);
      for (int i = 1; i < steps; i++){
        Point3 cur = BEZIER(a,cprev,cnext,b,startfracc + (float(i)/(steps-1))*between);
        curvedist += Distance(cur,last);
        if (curvedist > 0.0f){
          return (cur + last) * 0.5f;
        }
        last = cur;
      }
      return last;
    }

  }
};


//////////////////////////////////////////////////////////////////////////
// GridLayout


int  GridLayout::GetCornerVertex(const BitArray &edges, const BitArray &verts, const BitArray &faces,
                 BitArray &cornerverts, BitArray &cornerspecial)
{
  MNFace *face = m_mesh->F(0);
  int fcnt = m_mesh->FNum();

  int cornervert = -1;
  bool illegal = false;

  for (int f = 0; f < fcnt; f++,face++){
    if (!faces[f]) continue;
    // only quads allowed
    int deg = face->deg;

    // how many edges are part of border
    int bordercnt = 0;
    for (int i = 0; i < deg; i++){
      bordercnt += edges[face->edg[i]];
    }
    // island face (fully surrounded)
    if (bordercnt == deg) return -1;

    if (deg != 4){
      illegal = true;
      continue;
    }

    // in a quad there can be following corner cases
    // x - x
    // |   |  . case (isolated vert) no edge is on border
    // C - x
    //
    // x - x
    // +   |  L case (two border edges)
    // C + x
    //
    // x - x
    // +   +  U case  (three borders)
    // C + C

    // special    = 2 or more connected borderedges (L or U case)
    // cornervert = every vert not part of a borderedge
    //        or part of a connected borderedge
    //        (L, U, . case)


    for (int i = 0; i < deg; i++){
      int vid = face->vtx[i];

      if (verts[vid]){

        int vidprev = face->vtx[(i+deg-1)%deg];
        int vidnext = face->vtx[(i+1)%deg];

        BOOL nextin = verts[vidnext];
        BOOL previn = verts[vidprev];

        // first the simple cornerverts (non-borderedge)
        // if prev and next arent on border, then this vertex
        // is not part of a border edge
        if (!nextin && !previn){
          // isolated - case
          cornervert = vid;
          m_cornercnt++;
          cornerverts.Set(vid);
        }
        else{
          int prevedge = face->edg[(i+deg-1)%deg];
          int nextedge = face->edg[i];

          previn = edges[prevedge];
          nextin = edges[nextedge];

          if (bordercnt > 1 && nextin && previn){
            // next and prev are in
            // L or U - case
            cornervert = vid;
            m_cornercnt++;
            cornerverts.Set(vid);
            cornerspecial.Set(vid);
          }
        }
      }
    }
  }

  return illegal ? -1 : cornervert;
}


struct VertexAngle {
  float   angle;
  int     vid;
  int     idx;
};

static int VertexAngleSort(const void* el0, const void* el1)
{
  const VertexAngle* va = (const VertexAngle*)el0;
  const VertexAngle* vb = (const VertexAngle*)el1;

  return va->angle < vb->angle ? 1 : -1;
}

static int minDistances(const Tab<int> &borderverts, const BitArray &cornerverts, int idx)
{
  int count = borderverts.Count();
  for (int i = 1; i < count; i++){
    if (cornerverts[ borderverts[(idx + i) % count] ] ||
      cornerverts[ borderverts[(idx - i + count) % count] ]){
        return i;
    }
  }

  return 0;
}

static int minCornerDistances(const Tab<int> &borderverts, const BitArray &cornerverts, int distances[2])
{
  int count = borderverts.Count();
  distances[0] = count;
  distances[1] = count;
  int runner = 0;
  int found  = 0;
  int seed = -1;
  for (int i = 1; i < count * 2; i++){
    int vid = borderverts[i % count];
    if (cornerverts[ vid ]){
      if (found){
        int cmp = runner;
        for (int m = 0; m < 2; m++){
          if (cmp < distances[m]){
            std::swap(cmp,distances[m]);
            if (m == 0){
              seed = i;
            }
          }
        }
      }
      runner = 0;
      found  = 1;
    }
    runner += found;
  }

  return seed;
}

static void fillSides(const Tab<int> &borderverts, const BitArray &cornerverts, std::vector<int> &sides)
{
  sides.clear();

  int last = -1;
  int runner = 0;
  for (int i = 0; i < borderverts.Count(); i++,runner++){
    if (cornerverts[borderverts[i]]){
      sides.push_back(runner);
      runner = 0;
      last = i;
    }
  }

  if (!sides.empty()){
    sides[0] += borderverts.Count() - last;
  }

}

static int  findBestAngleFromTo(const Tab<VertexAngle>  vertexAngles, int from, int to)
{
  int   count = vertexAngles.Count();
  from = from;
  to   = to;

  int   lastKey = from % count;
  float lastAngle = -2.0;
  for (int i = from; i <= to; i++)
  {
    int idx = (count + i) % count;
    if (vertexAngles[ idx ].angle > lastAngle){
      lastKey = idx;
      lastAngle = vertexAngles[ idx ].angle;
    }
  }

  return lastKey;
}

static int  findBestAngle(const Tab<VertexAngle>  vertexAngles, int around, int width)
{
  int   count = vertexAngles.Count();
  int   lastKey = around % count;
  float lastAngle = -2.0;
  for (int i = -width; i <= width; i++)
  {
    int idx = (count + i + around) % count;
    if (vertexAngles[ idx ].angle > lastAngle){
      lastKey = idx;
      lastAngle = vertexAngles[ idx ].angle;
    }
  }

  return lastKey;
}


int GridLayout::TryFourCorners( const BitArray &edges, const BitArray &verts, const BitArray &faces, BitArray &cornerverts, BitArray &cornerspecial )
{
  int numVerts = m_borderverts.Count();

  // kill all special ones, we can only deal with single winded hole
  cornerverts &= ~cornerspecial;
  cornerspecial.ClearAll();

  // create sorted list of maximum angles
  Tab<VertexAngle>  vertexAngles;
  vertexAngles.SetCount(numVerts);

  for (int i = 0; i < numVerts; i++){
    int self = m_borderverts[i];
    int next = m_borderverts[(i + 1) % numVerts];
    int prev = m_borderverts[(i - 1 + numVerts) % numVerts];

    vertexAngles[i].vid = self;
    vertexAngles[i].idx = i;
    vertexAngles[i].angle = DotProd( (m_mesh->P(next) - m_mesh->P(self)).Normalize(),
                                     (m_mesh->P(prev) - m_mesh->P(self)).Normalize());
  }
  Tab<VertexAngle>  vertexAnglesOrig = vertexAngles;;
  vertexAngles.Sort(VertexAngleSort);

  m_cornercnt = cornerverts.NumberSet();
  if (m_cornercnt > 4){
    int left = m_cornercnt - 4;
    // kill the ones with least angles
    for (int i = numVerts-1; i >= 0 && left; i--){
      if ( cornerverts[vertexAngles[i].vid] ){
        cornerverts.Clear(vertexAngles[i].vid);
        left--;
      }
    }
  }
  else{
    // let's try to find optimal grids
    bool optimal = false;
    if (m_cornercnt == 2 || m_cornercnt == 3){
      int distances[2];
      int edges = numVerts + 1;
      int seed  = minCornerDistances(m_borderverts,cornerverts, distances);
      int rest;

      rest = edges - distances[0]*2;
      if (m_cornercnt == 2 && rest >= 2 && rest % 2 == 0){
        cornerverts.Set(m_borderverts[ (seed + rest/2) % numVerts ]);
        cornerverts.Set(m_borderverts[ (seed + rest/2 + distances[0]) % numVerts ]);

        m_cornercnt = 4;
        optimal = true;
      }

      rest = edges - (distances[0]*3 + distances[1]);
      if (m_cornercnt == 3 && rest >= 0){
        int from; 
        int to;
        if (cornerverts[m_borderverts[(seed + distances[1]) % numVerts]]){
          // forward  corner dist0 seed dist1 corner
          from = seed + distances[1];
          to   = seed - distances[0]*2 + numVerts;
        }
        else{
          // backward corner dist1 corner dist0 seed
          from = seed + distances[0];
          to   = seed - (distances[0]+distances[1]) + numVerts;
        }

        int forth = findBestAngleFromTo(vertexAnglesOrig,from,to);
        cornerverts.Set(m_borderverts[forth]);
        
        m_cornercnt = 4;
        optimal = true;
      }
    }

    if (!optimal){
      if (!m_cornercnt){
        // add highest angle
        m_cornercnt++;
        cornerverts.Set(vertexAngles[0].vid);
      }
      // we either have 1,2 or 3 corners already
      // find highest angle with minimum distance of 2 to other points?
      int distance = 2;
      while (m_cornercnt != 4 && distance){
        for (int i = 0; i < numVerts && m_cornercnt < 4; i++){
          if (!cornerverts[ vertexAngles[i].vid ] &&
            minDistances(m_borderverts,cornerverts, vertexAngles[i].idx) >= distance ){
              cornerverts.Set(vertexAngles[i].vid);
              m_cornercnt++;
          }
        }
        distance--;
      }
    }
  }

  std::vector<int>  sides;

  if (m_cornercnt == 4){
    // test if cap can fill it, otherwise use alternative logic
    fillSides(m_borderverts,cornerverts,sides);
    if (GridCap::ValidLayout(sides)){
      // return random seed
      for (int i = 0; i < numVerts; i++){
        if (cornerverts[ m_borderverts[i] ])
          return m_borderverts[i];
      }
    }
  }

  {
    int first = vertexAngles[0].idx;

    // add highest angle
    // find opposites
    int searchWindow = numVerts/3;
    int half         = numVerts/2;

    while (searchWindow > 0){
      cornerverts.ClearAll();
      cornerverts.Set(m_borderverts[first]);

      int third = findBestAngle(vertexAnglesOrig,first + half,searchWindow);
      cornerverts.Set(m_borderverts[third]);

      int second = findBestAngle(vertexAnglesOrig,(first + third + 1) / 2,searchWindow);
      int forth  = findBestAngle(vertexAnglesOrig,(first + third + 1) / 2 + half,searchWindow);
      cornerverts.Set(m_borderverts[second]);
      cornerverts.Set(m_borderverts[forth]);

      
      fillSides(m_borderverts,cornerverts,sides);
      m_cornercnt = sides.size();
      if (GridCap::ValidLayout(sides)){
        return m_borderverts[first];
      }

      searchWindow--;
    }

  }

  return -1;
}



IPoint2 IPoint2Min(IPoint2 &a, IPoint2 &b)
{
  return IPoint2(min(a.x,b.x),min(a.y,b.y));
}
IPoint2 IPoint2Max(IPoint2 &a, IPoint2 &b)
{
  return IPoint2(max(a.x,b.x),max(a.y,b.y));
}

BOOL GridLayout::BuildGridSides(int cv, const BitArray &edges, const BitArray &cornerverts, const BitArray &cornerspecial)
{
  // start a random cornervert, walk along border edges till next corner
  // do until no cornervert left
  int rev = cornerspecial[cv] ? -1 : 1;

  BOOL  hactive = TRUE;
  int   angle = 0;

  // last is same as first
  int vertcnt = m_borderverts.Count();
  int edgecnt = m_borderedges.Count();

  // must be closed
  SM_ASSERT(edgecnt == vertcnt);

  // find cornervert
  int curidx = -1;
  for (int i = 0; i < vertcnt; i++){
    if (m_borderverts[i] == cv){
      curidx = i;
    }
  }
  SM_ASSERT(curidx >= 0);

  // now generate the edgestrips, and give them signs
  // walking along edge and turning 90° (left or right depending
  // on border vert, gives each edge a horizontal and vertical direction

  IPoint2 coordhigh(-5000,-5000);
  IPoint2 coordlow(5000,5000);
  IPoint2 coord(0,0);
  IPoint2 dirsign(1,1);

  m_sides.clear();
  int cvcnt = m_cornercnt;
  while (cvcnt > 0){
    int v = cv;

    Side gside;
    gside.size = 0;
    gside.startidx = curidx;
    gside.startspecial = cornerspecial[v];
    gside.coord = coord;

    // run to next corner
    do {
      gside.size++;

      curidx = (curidx+1)%edgecnt;
      v = m_borderverts[curidx];
    }while (!cornerverts[v]);

    // handle the turning at the end of a edge
    angle %=  4;

    if (angle == 3)   angle = -1;
    if (angle == -3)  angle =  1;

    if (angle == 0)         dirsign.x =  1*rev;
    else if (angle == 2 || 
             angle == -2)   dirsign.x = -1*rev;
    else if (angle == 1)    dirsign.y =  1*rev;
    else if (angle == -1)   dirsign.y = -1*rev;

    if (cornerspecial[v]) angle += 1;
    else                  angle -= 1;

    // set direction
    if (hactive)  gside.dir = IPoint2(dirsign.x,0);
    else      gside.dir = IPoint2(0,dirsign.y);

    // update coords
    coord  += gside.dir * gside.size;
    coordhigh = IPoint2Max(coordhigh,coord);
    coordlow  = IPoint2Min(coordlow,coord);

    gside.sign = dirsign;

    // add side
    m_sides.push_back(gside);

    // prep for next
    hactive = !hactive;
    cv = v;
    cvcnt--;
  }

  int sidecnt = (int)m_sides.size();
  Side *gs = &m_sides[0];
  for (int i = 0 ; i < sidecnt; i++,gs++){
    gs->coord -= coordlow;
  }
  coordhigh -= coordlow;

  m_dim = coordhigh + IPoint2(1,1);
  m_fillcnt = m_dim.x*m_dim.y;

  m_sidecnt = sidecnt;

  // we did not end at startpoint (0,0)
  if (coord != IPoint2(0,0)){
    return FALSE;
  }

  return TRUE;
}

BOOL GridLayout::Init(MNMesh  *mesh, Tab<int> *curloop)
{
  BOOL result = FALSE;
  m_useinner = FALSE;
  m_mesh = mesh;

  BitArray faces(mesh->FNum());
  BitArray edges(mesh->ENum());
  BitArray verts(mesh->VNum());

  BitArray cornerverts(mesh->VNum());
  BitArray cornerspecial(mesh->VNum());


  int seededge = *curloop->Addr(0);

  // get border and fill surroundings
  mesh->BorderFromEdge(seededge,edges);

  BitArrayEdgeToVertex etov(mesh,verts);
  SM_BITARRAY_ENUMSET(edges,etov,BitArrayEdgeToVertex);
  BitArrayVertexToFace vtof(mesh,faces);
  SM_BITARRAY_ENUMSET(verts,vtof,BitArrayVertexToFace);

  // build loop
  m_borderedges = *curloop;
  int edgecnt = m_borderedges.Count();

  m_borderverts.SetCount(edgecnt);
  for (int i = 0; i < edgecnt; i++){
    m_borderverts[i] = mesh->E(m_borderedges[i])->v1;
  }
  m_edgecnt = edgecnt;

  m_cornercnt = 0;
  m_sidecnt = 0;
  int seedcorner = GetCornerVertex(edges,verts,faces,
                  cornerverts,cornerspecial);

  BitArray cornervertsorig = cornerverts;

  bool  tryAgain = true;
  int   tryCount = 0;
  while (tryAgain && tryCount <= 1){
    tryAgain = false;

    // cases:
    // 4 corners, can cap/fill
    // >4 corners, check for fill config
    // if neither of those, try four corner generation
    if (seedcorner < 0 || m_cornercnt < 4){
      // try new seed corner
      seedcorner = TryFourCorners(edges,verts,faces,
        cornerverts,cornerspecial);
      tryAgain = true;
      tryCount++;
    }
    else
    {
      if (BuildGridSides(seedcorner,edges,cornerverts,cornerspecial)){
        return TRUE;
      }
      else if(m_cornercnt != 4) {
        seedcorner = -1;
        tryAgain = true;
        cornerverts = cornervertsorig;
      }
    }
  }

  return FALSE;
}

void GridLayout::BuildPoints()
{
  if (m_gridouterpts.size()) return;

  int sidecnt = m_sidecnt;

  // run thru all sides, and generate proper vertices
  int numout = 0;
  for (int s = 0; s < sidecnt; s++){
    GridLayout::Side &gside = GetSide(s);

    IPoint2 coord = gside.coord;
    float sidelength = 0.0f;

    int prevvid = GetVertex(gside.startidx-1);
    int nextvid = -1;
    gside.outstart    = numout;
    m_gridouterpts.resize(m_gridouterpts.size() + gside.size);

    Point2  fcoord(float(coord.x),float(coord.y));
    Point2  fdir(float(gside.dir.x),float(gside.dir.y));

    for (int i = 0; i < gside.size; i++){
      int curidx = gside.startidx+i;

      BorderPoint opoint;
      opoint.special = 1.0;
      opoint.coord = coord;
      opoint.sign = gside.sign;
      opoint.vid  = GetVertex(curidx);
      nextvid     = GetVertex(curidx+1);

      const Tab<int> &edges = m_mesh->vedg[opoint.vid];
      int edgecnt = edges.Count();

      // find opposite vertices (inwards)
      // and raise length
      if (i == 0 && (edgecnt > 3 || edgecnt  < 3))
      { // first is corner 
        if (gside.startspecial){
          opoint.special = -1.0f;
          //  next is always same dir
          if (gside.dir.x){
            // next is horizontal
            opoint.outer[0]   = m_mesh->P(nextvid);
            opoint.outer[1]   = m_mesh->P(prevvid);
          }
          else{
            // next is vertical
            opoint.outer[0]   = m_mesh->P(prevvid);
            opoint.outer[1]   = m_mesh->P(nextvid);
          }
        }
        else{
          if (!m_mesh->GetFlag(MN_MESH_VERTS_ORDERED)){
            m_mesh->OrderVert(opoint.vid);
          }
          int e;
          for ( e = 0; e < edgecnt; e++){
            int opvid = m_mesh->E(edges[e])->OtherVert(opoint.vid);
            if (opvid == prevvid)
              break;
          }

          int opvidprev = m_mesh->E(edges[(e + 2) % edgecnt])->OtherVert(opoint.vid);
          int opvidnext = m_mesh->E(edges[(e + 3) % edgecnt])->OtherVert(opoint.vid);
          if (nextvid == opvidnext){
            opvidnext = m_mesh->E(edges[(e + 1) % edgecnt])->OtherVert(opoint.vid);
          }

          if (gside.dir.x){
            // next is horizontal
            opoint.outer[0]   = m_mesh->P(opvidnext);
            opoint.outer[1]   = m_mesh->P(opvidprev);
          }
          else{
            // next is vertical
            opoint.outer[0]   = m_mesh->P(opvidprev);
            opoint.outer[1]   = m_mesh->P(opvidnext);
          }
        }
      }
      else
      { // non corner, inbetween
        // any outgoing edge that is not on border
        // should be fine
        Point3  avg(0,0,0);
        float   avgNum = 0.0f;
        for (int e = 0; e < edgecnt; e++){
          int opvid = m_mesh->E(edges[e])->OtherVert(opoint.vid);
          if (opvid != nextvid && opvid != prevvid){
            avg += m_mesh->P(opvid);
            avgNum += 1.0f;
          }
        }

        if (avgNum > 0.0){
          avg /= avgNum;
        }
        else{
          avg = m_mesh->P(opoint.vid);
        }

        opoint.outer[0]   = avg;
        opoint.outer[1]   = avg;

        // add length to prev
        if (i != 0 ){
          sidelength += Distance(m_mesh->P(prevvid),m_mesh->P(opoint.vid));
        }
      }
      opoint.fracc      = sidelength;
      opoint.inner      = m_mesh->P(opoint.vid);

      // advance
      m_gridouterpts[gside.outstart + i] = opoint;
      numout++;

      coord += gside.dir;
      prevvid = opoint.vid;
    }
    // add last distance
    sidelength += Distance(m_mesh->P(prevvid),m_mesh->P(nextvid));

    // normalized position
    float scale = float(gside.size) / sidelength;
    for (int i = 0; i < gside.size; i++){
      BorderPoint &opt = m_gridouterpts[gside.outstart+i];
      opt.fcoord  = fcoord + (fdir * scale * opt.fracc);
      opt.fracc  /= sidelength;
    }
  }

}

void GridLayout::GetSideMatrix(GridLayout::Side &gside, int pt, Matrix3 &orientation)
{
  bool last = pt == gside.size;
  int opside = gside.dir.x ? 1 : 0;

  BorderPoint& ptself = GetBorderPoint(gside.outstart + pt);
  BorderPoint& ptnext = GetBorderPoint(gside.outstart + (last ? pt -1 : pt + 1));

  Point3 tonext  = Normalize(ptnext.inner - ptself.inner) * (last ? -1.0f : 1.0f);
  Point3 toouter = Normalize(ptself.outer[opside] - ptself.inner);

  orientation.SetRow(0,tonext);
  orientation.SetRow(2,Normalize(CrossProd(tonext,toouter)));
  orientation.SetRow(1,Normalize(CrossProd(tonext,orientation.GetRow(2))));
  orientation.SetRow(3,Point3(0,0,0));
}

void GridLayout::BuildAndUseInnerTangents()
{
  // inner tangents
  // for each side find relative start/end rotation

  // apply to local matrices

  for (int s = 0; s < m_sidecnt; s++){
    GridLayout::Side &gside = GetSide(s);
    int opside = gside.dir.x ? 1 : 0;

    Matrix3 matbegin;
    Matrix3 matend;
    GetSideMatrix(gside,0,matbegin);
    GetSideMatrix(gside,gside.size,matend);
    
    Point3 dirbegin = -(GetBorderPoint(gside.outstart -1).inner - GetBorderPoint(gside.outstart).inner) * GetBorderPoint(gside.outstart).special;
    Point3 dirend   = -(GetBorderPoint(gside.outstart + gside.size + 1).inner - GetBorderPoint(gside.outstart + gside.size).inner) * GetBorderPoint(gside.outstart + gside.size).special;

    GetBorderPoint(gside.outstart).tangent[opside] = GetBorderPoint(gside.outstart).inner + dirbegin;
    GetBorderPoint(gside.outstart+ gside.size).tangent[opside] = GetBorderPoint(gside.outstart + gside.size).inner + dirend;

    float lenbegin = dirbegin.Length();
    float lenend   = dirend.Length();

    dirbegin = Normalize(dirbegin);
    dirend   = Normalize(dirend);

#if 0
    Matrix3 relbegin(1);
    relbegin.SetRow(2,Normalize(CrossProd(matbegin.GetRow(0),dirbegin)));
    relbegin.SetRow(1,dirbegin);
    relbegin.SetRow(0,Normalize(CrossProd(relbegin.GetRow(2),dirbegin)));

    Matrix3 relend(1);
    relend.SetRow(2,Normalize(CrossProd(matend.GetRow(0),dirend)));
    relend.SetRow(1,dirend);
    relend.SetRow(0,Normalize(CrossProd(relend.GetRow(2),dirend)));

    relbegin = relbegin * Inverse(matbegin);
    relend = relend * Inverse(matend);

    Quat relbeginq (relbegin);
    Quat relendq (relend);
#endif
    Point3 reldirbegin = VectorTransform(dirbegin,Inverse(matbegin));
    Point3 reldirend   = VectorTransform(dirend,Inverse(matend));



    for (int i = 1; i < gside.size; i++){
      Matrix3 mat;
      float fracc = GetBorderPoint(gside.outstart + i).fracc;
      float len = LERP(lenbegin,lenend,fracc);
#if 0
      Quat rel = Slerp(relbegin,relend,fracc);
      rel.MakeMatrix(mat);
      Point3 dir = VectorTransform(Point3(0.0f,len,0.0f),mat);
#else
      Point3 dir = LERP(reldirbegin,reldirend,fracc);
      dir = Normalize(dir) * len;      
#endif
      GetSideMatrix(gside,i,mat);
      dir = VectorTransform(dir,mat);

      GetBorderPoint(gside.outstart + i).tangent[0] = 
      GetBorderPoint(gside.outstart + i).tangent[1] = GetBorderPoint(gside.outstart + i).inner + dir;
    }
  }

  m_useinner = TRUE;
}


inline void GridLayout::InterpolateSide( int sideIdx, float fracc, Point3 &inner, Point3 &outer ) const
{
  const Side& side = GetSide(sideIdx);

  int from = 0;
  bool islast = true;
  for (int i = 0; i < side.size; i++)
  {
    from = side.outstart + i;
    if ( i < side.size - 1 && m_gridouterpts[from + 1].fracc > fracc){
      islast = false;
      break;
    }
  }

  const BorderPoint& ofrom = m_gridouterpts[ from ];
  const BorderPoint& oto   = m_gridouterpts[(from + 1) % m_gridouterpts.size()];
  int opside = side.dir.x ? 1 : 0;

  float tofracc = islast ? 1.0f : oto.fracc;
  fracc = (fracc - ofrom.fracc) / (tofracc - ofrom.fracc);

  inner = LERP(ofrom.inner, oto.inner, fracc);

  if (m_useinner){
    outer = LERP(ofrom.tangent[opside], oto.tangent[opside], fracc);
  }
  else{
    outer = LERP(ofrom.outer[opside], oto.outer[opside], fracc);
  }
}

//////////////////////////////////////////////////////////////////////////
// GridFill


void GridFill::CreateMeshVertex(InnerPoint &gin, OuterPoint *from, OuterPoint *to)
{
  Point2 fracc;

  Line2DIntersect(gin.from->fcoord,gin.to->fcoord,from->fcoord,to->fcoord,fracc);

  const Point3 &pfromouth = m_gip->m_inner ? gin.from->tangent[0] : gin.from->outer[0];
  const Point3 &pfromh    = gin.from->inner;
  const Point3 &ptoh      = gin.to->inner;
  const Point3 &ptoouth   = m_gip->m_inner ? gin.to->tangent[0] : gin.to->outer[0];

  const Point3 &pfromoutv = m_gip->m_inner ? from->tangent[1] : from->outer[1];
  const Point3 &pfromv    = from->inner;
  const Point3 &ptov      = to->inner;
  const Point3 &ptooutv   = m_gip->m_inner ? to->tangent[1] : to->outer[1];

  Point3 pos =  m_gip->Run(fracc, pfromouth, pfromh, ptoh, ptoouth, pfromoutv, pfromv, ptov, ptooutv );

  gin.vid += m_startvid;
  m_mesh->V(gin.vid)->p = pos;
}
void GridFill::CreateMeshVertexLine(OuterPoint *from, OuterPoint *to, int startidx, int curidx, int stride)
{
  int cnt = (curidx-startidx)/stride;
  int len = cnt-1;

  for (int i = 1; i < cnt; i++){
    int idx = startidx + (i*stride);
    Point &gp = m_grid[idx];
    SM_ASSERT(gp.isinner);

    CreateMeshVertex(m_gridinnerpts[gp.inneridx],from,to);
  }
}
void GridFill::CreateInnerPointLine(OuterPoint *from, OuterPoint *to, int startidx, int curidx, int stride)
{
  int cnt = (curidx-startidx)/stride;

  for (int i = 1; i < cnt; i++){
    int idx = startidx + (i*stride);
    Point &gp = m_grid[idx];
    InnerPoint  ipoint;

    ipoint.vid = m_numinner;
    ipoint.from = from;
    ipoint.to = to;

    m_gridinnerpts.push_back(ipoint);

    gp.isinner = TRUE;
    gp.inneridx = m_numinner;

    m_numinner++;
  }
}
void GridFill::CreateGridMesh()
{
  // create inbetween points in first pass
  // then create mesh

  Point *gp;
  OuterPoint *from = NULL;
  int startidx = -1;

  // first pass horizontal
  // creates inner points
  m_numinner = 0;

  int stride = 1;
  for (int y = 0; y < m_dim.y; y++){
    for (int x = 0; x < m_dim.x; x++){
      int idx = grididx(x,y);
      OuterPoint *cur = m_grid[idx].outer;
      if (cur){
        if (from && idx-startidx > stride && cur->sign.y == -1){
          // at least one inbetween and opposite side
          CreateInnerPointLine(from,cur,startidx,idx,stride);
          from = NULL;
        }
        else if (cur->sign.y == 1){
          from = cur;
          startidx = idx;
        }
        else{
          from = NULL;
        }
      }
    }
    from = NULL;
  }

  // for each inner point we will need a new vertex
  m_startvid = m_mesh->AppendNewVerts(m_numinner);


  // vertical pass
  // creates vertices
  stride = m_dim.x;
  for (int x = 0; x < m_dim.x; x++){
    for (int y = 0; y < m_dim.y; y++){
      int idx = grididx(x,y);
      gp = &m_grid[idx];
      OuterPoint *cur = gp->isinner ? NULL : gp->outer;
      if (cur){
        if (from && idx-startidx > stride && cur->sign.x == 1){
          // at least one inbetween and opposite side
          CreateMeshVertexLine(from,cur,startidx,idx,stride);
          from = NULL;
        }
        else if (cur->sign.x == -1){
          from = cur;
          startidx = idx;
        }
        else{
          from = NULL;
        }
      }
    }
  }

  // now create output grid with all vertexids
  int totalcnt = m_layout->m_fillcnt;
  int *outvids = new int[totalcnt];
  int *outvid = outvids;

  gp = m_grid; //&m_grid[totalcnt-1];
  for (int i = 0; i < totalcnt; i++,outvid++,gp++){
    *outvid = gp->isinner ? m_gridinnerpts[gp->inneridx].vid :
                ((gp->outer) ? gp->outer->vid : -1);
  }

  // fill mesh with quads
  int facecnt = 0;
  int startfid = MNMesh_fillQuadRectangle(m_mesh,outvids,m_dim.x,m_dim.y,facecnt);
  for (int i = 0; i < facecnt; i++)
    m_mesh->F(i+startfid)->ClearFlag(MN_SEL);

  delete[] outvids;
}

void GridFill::BuildGridOuterPoints()
{
  m_layout->BuildPoints();

  // setup grid, now that pointers dont change anymore
  m_grid = new Point[m_layout->m_fillcnt];
  memset(m_grid,0,sizeof(Point)*m_layout->m_fillcnt);

  int outercnt = (int)m_layout->m_gridouterpts.size();
  OuterPoint *opt = &m_layout->m_gridouterpts[0];
  for (int i = 0; i < outercnt; i++,opt++){
    m_grid[grididx(opt->coord)].outer = opt;
  }
}

void GridFill::Run(GridLayout *gridlayout, GridInterpolator *gip)
{
  Clear();
  SM_ASSERT(gridlayout && gip);

  m_mesh = gridlayout->m_mesh;
  m_gip = gip;
  m_layout = gridlayout;
  m_dim = gridlayout->m_dim;

  BuildGridOuterPoints();
  if (gip->m_inner){
    m_layout->BuildAndUseInnerTangents();
  }
  CreateGridMesh();

  Clear();
}


void GridFill::Clear()
{
  if (m_grid)
    delete[] m_grid;

  m_grid = NULL;
  m_layout = NULL;
  m_mesh = NULL;
  m_gip = NULL;

  m_gridinnerpts.clear();
}


//////////////////////////////////////////////////////////////////////////

bool GridCap::ValidLayout( const GridLayout &gridlayout )
{
  std::vector<int>  inputsides;
  for (int s = 0; s < gridlayout.m_sidecnt; s++){
    inputsides.push_back(gridlayout.GetSide(s).size);
  }

  return ValidLayout(inputsides);
}

bool GridCap::ValidLayout( const std::vector<int>  &inputsides )
{
  if (inputsides.size() != 4)
    return false;

  TConfig cfg;
  int sizes[4];
  int sides[2];
  GetSidesSizes(inputsides,sides,sizes);
  GridCapConfig gccfg;
  gccfg.pole.x = -1;
  gccfg.pole.y = -1;
  cfg.Init(sizes,gccfg);

  return cfg.type != FILL_INVALID;
}

void GridCap::Run( GridLayout *gridlayout, GridInterpolator *gip, const GridCapConfig &cfg, Tab<int> *newFaces )
{
  m_layout = gridlayout;
  m_gip    = gip;
  m_mesh   = gridlayout->m_mesh;

  m_layout->BuildPoints();
  if (gip->m_inner){
    m_layout->BuildAndUseInnerTangents();
  }

  std::vector<int>  inputsides;
  for (int s = 0; s < gridlayout->m_sidecnt; s++){
    inputsides.push_back(gridlayout->GetSide(s).size);
  }

  Topo topo;
  InitSides(topo,inputsides,cfg);
  if (topo.cfg.type == FILL_SPLIT){
    Topo upper;
    Topo lower;
    Split(topo,upper,lower);
    Fill(upper);
    Fill(lower);
    Merge(topo,upper,lower);
  }
  else{
    Fill(topo);
  }
  CreateMesh(topo,newFaces);
}

void GridCap::GetSidesSizes( const std::vector<int>&inputsides, int sides[2], int sizes[4])
{
  int diff0 = inputsides[0] - inputsides[2];
  int diff1 = inputsides[1] - inputsides[3];

  sides[0] = diff0 >= 0 ? 0 : 2;
  sides[1] = diff1 == 0 ? sides[0] + 1 : (diff1 > 0 ? 1 : 3);

  if (abs(diff0) < abs(diff1) ){
    int tmp = sides[0];
    sides[0] = sides[1];
    sides[1] = tmp;
  }

  for (int i = 0; i < 2; i++){
    int m = sides[i];
#if 1
    sizes[i] = inputsides[(m)];
    sizes[(i + 2) % 4] = inputsides[((m + 2) % 4)];
#else
    sizes[m] = inputsides[(m)];
    sizes[(m + 2) % 4] = inputsides[((m + 2) % 4)];
#endif
  }
}

void GridCap::InitSides( Topo &topo, const std::vector<int> &inputsides, const GridCapConfig &cfg )
{
  //////////////////////// 
  //  side   q:3
  //     0,0______
  //       |      |
  //  m:0  |      |  n:2
  //       |______|
  //
  //         p:1

  // m > n , p > q
  // m - n > p - q 
  // find sides

  topo.sides.SetCount(4);

  int sides[2];
  int sizes[4];
  GetSidesSizes(inputsides,sides,sizes);

  // check if reversed
  m_layoutReverse = sides[1] != ((sides[0] + 1) % 4);

  for (int i = 0; i < 2; i++){
    int m = sides[i];
    topo.sides[i].layoutSide = m;
    topo.sides[i].size = sizes[i];
    topo.sides[(i + 2) % 4].layoutSide = (m + 2) % 4;
    topo.sides[(i + 2) % 4].size = sizes[(i + 2) % 4];
  }

  topo.cfg.Init(sizes,cfg);
}


void GridCap::Split( const Topo &topo, Topo &upper, Topo &lower )
{
  int alphaN = topo.sides[1].size - 2;
  int betaN  = topo.sides[3].size - 2;

  int sigma  = topo.cfg.summed - (alphaN + betaN);
  int Mpos   = topo.cfg.fractions[0][0] + betaN + sigma/2;
  int ME7    = (sigma+1)/2 + 2;


}


void GridCap::TConfig::Init( int sizes[4], const GridCapConfig &gccfg)
{
  // find out what case we have
  int diff[2] = {
    sizes[0] - sizes[2],
    sizes[1] - sizes[3],
  };

  int totalsize = sizes[0] + sizes[1] + sizes[2] + sizes[3];

  // even, uneven, equal
  if (diff[0] == diff[1]){
    type = FILL_EQUAL;
  }
  else if ((totalsize) % 2){
    type = FILL_UNEVEN;
  }
  else {
    type = FILL_EVEN;
  }

  // alpha + beta = diff0
  // alpha - beta = diff1
  // beta  = alpha - diff1
  // alpha - diff1 + alpha = diff0
  // alpha 2 = diff0 + diff1

  int alpha2 = (diff[0] + diff[1]);
  int beta2  = alpha2 - 2*diff[1];

  summed  = (alpha2 + beta2)/2;
  alpha   = (alpha2 + 1)/2; // round up
  beta    = (beta2 + 1)/2; // round up

  int gaps[2] = {
    summed,
    alpha,
  };

  int  target[2] = {gccfg.pole.x,gccfg.pole.y};
  bool centered[2] = {gccfg.pole.x < 0, gccfg.pole.y < 0};

  bool unsolved[2] = {true,true};
  for (int i = 0; i < 2; i++){
    fractions[i][0] = -1;
    fractions[i][1] = -1;

    int deltaDiff = 1000000;
    for (int f0 = 0; f0 <= sizes[i]; f0++){
      int f1 = sizes[i] - gaps[i] - f0;
      int delta = abs(f0-f1);
      if (f1 >= 0 && delta < deltaDiff){
        deltaDiff = delta;
        fractions[i][0] = f0;
        fractions[i][1] = f1;
        unsolved[i] = false;
        if (!centered[i]){ //  && diff[1] != 0
          // try to get fractions towards target values
          if (i == 1){
            if (fractions[i][0] >= 0){
              if (fractions[i][0] >= target[i]){
                break;
              }
              else{
                deltaDiff = 1000000;
              }
            }
          }
          else{
            if (fractions[i][1] > 0){
              if (fractions[i][0] >= target[i]){
                break;
              }
              else{
                deltaDiff = 1000000;
              }
            }
          }
        }
      }
      if (delta > deltaDiff)
        break;
    }

  }
#if 0
  if (!fractions[0][1] && !fractions[1][0]){
    type = FILL_INVALID;
    return;
  }
#endif

  if (unsolved[0] || unsolved[1]){
    if (unsolved[0]){
      type = FILL_INVALID;
      return;
    }

    if (sizes[2] == fractions[0][0] + fractions[0][1]){
      // TODO add more checks split case
      type = FILL_INVALID;
      return;
    }
    else {
      type = FILL_INVALID;
      return;
    }
    
  }

  //////////////////////////////////////////////////
  //                   beta
  //
  //       C0________E6__E5_____C3
  //  f00   |   0     | 1 |  2   |
  //       E0________G1\  |      |
  //  a     |   6   /    \GP_____E7
  //  +    E1______GN  8 /|      |
  //  b     |   7    \  / |  5   |
  //       E2________G2   |      |
  //  f01   |   3    | 4  |      |
  //       C1________E3__E4______C2
  //
  //           f10    alpha f11
  //
  // triangle:
  //   sector 9 == 7+6


  sectorsizes[0] = fractions[0][0] * fractions[1][0];
  sectorsizes[1] = fractions[0][0] * beta;
  sectorsizes[2] = fractions[0][0] * fractions[1][1];
  sectorsizes[3] = fractions[0][1] * fractions[1][0];
  sectorsizes[4] = fractions[0][1] * alpha;
  sectorsizes[5] = fractions[0][1] * fractions[1][1];
  sectorsizes[6] = alpha * fractions[1][0];
  sectorsizes[7] = beta  * fractions[1][0];
  sectorsizes[8] = alpha * beta;
  sectorsizes[9] = summed * fractions[1][0];
}


void GridCap::Fill( Topo &topo )
{
  const int (&sectorsizes)[10] = topo.cfg.sectorsizes;
  const int (&fractions)[2][2] = topo.cfg.fractions;
  topo.points.SetCount(TPTS);


  //////////////////////////////////////////////////
  //                   beta
  //
  //       C0________E6__E5_____C3
  //  f00   |   0     | 1 |  2   |
  //       E0________G1\  |      |
  //  a     |   6   /    \GP_____E7
  //  +    E1______GN  8 /|      |
  //  b     |   7    \  / |  5   |
  //       E2________G2   |      |
  //  f01   |   3    | 4  |      |
  //       C1________E3__E4______C2
  //
  //           f10    alpha f11
  //

  // fill in corners
  MakeSidePoint(topo,topo.points[TPT_CORNER0],0,0);
  MakeSidePoint(topo,topo.points[TPT_CORNER1],1,0);
  MakeSidePoint(topo,topo.points[TPT_CORNER2],2,0);
  MakeSidePoint(topo,topo.points[TPT_CORNER3],3,0);

  // legal to work if some fractions are 0
  MakeSidePoint(topo,topo.points[TPT_E0],0,fractions[0][0]);
  MakeSidePoint(topo,topo.points[TPT_E1],0,fractions[0][0] + topo.cfg.alpha);
  MakeSidePoint(topo,topo.points[TPT_E2],0,fractions[0][0] + topo.cfg.summed);
  MakeSidePoint(topo,topo.points[TPT_E3],1,fractions[1][0]);
  MakeSidePoint(topo,topo.points[TPT_E4],1,fractions[1][0] + topo.cfg.alpha);
  MakeSidePoint(topo,topo.points[TPT_E7],2,fractions[0][1]);
  MakeSidePoint(topo,topo.points[TPT_E5],3,fractions[1][1]);
  MakeSidePoint(topo,topo.points[TPT_E6],3,fractions[1][1] + topo.cfg.beta);



  // create inbetween points
  {
    Point2 points[4];
    int    weights[4];

    #define TPCOORD(what)  topo.points[what].vert.coord

    if (fractions[0][0] && fractions[0][1] && fractions[1][1] && !(topo.cfg.type == FILL_EQUAL && !fractions[1][0])){
      // GP exists
      points[0] = TPCOORD(TPT_E1) + TPCOORD(TPT_E5) - TPCOORD(TPT_CORNER0);
      points[1] = TPCOORD(TPT_E1) + TPCOORD(TPT_E4) - TPCOORD(TPT_CORNER1);
      points[2] = TPCOORD(TPT_E4) + TPCOORD(TPT_E7) - TPCOORD(TPT_CORNER2);
      points[3] = TPCOORD(TPT_E5) + TPCOORD(TPT_E7) - TPCOORD(TPT_CORNER3);
      weights[0] = sectorsizes[0] + sectorsizes[1] + sectorsizes[6] + sectorsizes[8];
      weights[1] = sectorsizes[3] + sectorsizes[5] + sectorsizes[7] + sectorsizes[8];
      weights[2] = sectorsizes[5];
      weights[3] = sectorsizes[2];
      CreateInnerPoint(topo,topo.points[TPT_GP],points,weights);
    }
    else{
      if (!fractions[1][1]){
        topo.points[TPT_GP] = topo.points[TPT_E7];
      }
      else if (fractions[0][0] && !fractions[0][1]){
        topo.points[TPT_GP] = topo.points[TPT_E4];
      }
      else if (fractions[0][1] && !fractions[0][0]){
        topo.points[TPT_GP] = topo.points[TPT_E5];
      }
      else {
        topo.points[TPT_GP] = topo.points[TPT_E0];
      }
    }

    if (fractions[0][0] && fractions[1][0] && topo.cfg.beta){
      // G1 exists
      points[0] = TPCOORD(TPT_E0) + TPCOORD(TPT_E6) - TPCOORD(TPT_CORNER0);
      points[1] = TPCOORD(TPT_E0) + TPCOORD(TPT_E3) - TPCOORD(TPT_CORNER1);
      points[2] = TPCOORD(TPT_E3) + TPCOORD(TPT_E7) - TPCOORD(TPT_CORNER2);
      points[3] = TPCOORD(TPT_E6) + TPCOORD(TPT_E7) - TPCOORD(TPT_CORNER3);
      weights[0] = sectorsizes[0];
      weights[1] = sectorsizes[3] + sectorsizes[9] + sectorsizes[8];
      weights[2] = sectorsizes[4] + sectorsizes[5] + sectorsizes[8];
      weights[3] = sectorsizes[1] + sectorsizes[2] + sectorsizes[8];
      CreateInnerPoint(topo,topo.points[TPT_G1],points,weights);
    }
    else{
      // same as E6 or E0 or GP
      if (!topo.cfg.beta){
        topo.points[TPT_G1] = topo.points[TPT_GP];
      }
      else if (fractions[0][0]){
        topo.points[TPT_G1] = topo.points[TPT_E0];
      }
      else{
        topo.points[TPT_G1] = topo.points[TPT_E6];
      }
    }

    if (fractions[0][1] && fractions[1][0]){
      // G2 exists
      points[0] = TPCOORD(TPT_E2) + TPCOORD(TPT_E6) - TPCOORD(TPT_CORNER0);
      points[1] = TPCOORD(TPT_E2) + TPCOORD(TPT_E3) - TPCOORD(TPT_CORNER1);
      points[2] = TPCOORD(TPT_E6) + TPCOORD(TPT_E7) - TPCOORD(TPT_CORNER3);
      points[3] = TPCOORD(TPT_E3) + TPCOORD(TPT_E7) - TPCOORD(TPT_CORNER2);
      weights[0] = sectorsizes[0] + sectorsizes[9] + sectorsizes[8];
      weights[1] = sectorsizes[3];
      weights[2] = sectorsizes[1] + sectorsizes[2] + sectorsizes[8];
      weights[3] = sectorsizes[4] + sectorsizes[5] + sectorsizes[8];
      CreateInnerPoint(topo,topo.points[TPT_G2],points,weights);
    }
    else{
      if (fractions[0][1]){
        topo.points[TPT_G2] = topo.points[TPT_E2];
      }
      else{
        topo.points[TPT_G2] = topo.points[TPT_E3];
      }
    }

    if (fractions[1][0] && topo.cfg.type == FILL_EVEN){
      // GN exists
      points[0] = TPCOORD(TPT_E1) + TPCOORD(TPT_E6) - TPCOORD(TPT_CORNER0);
      points[1] = TPCOORD(TPT_E1) + TPCOORD(TPT_E3) - TPCOORD(TPT_CORNER1);
      points[2] = TPCOORD(TPT_E3) + TPCOORD(TPT_E7) - TPCOORD(TPT_CORNER2);
      points[3] = TPCOORD(TPT_E6) + TPCOORD(TPT_E7) - TPCOORD(TPT_CORNER3);
      weights[0] = sectorsizes[0] + sectorsizes[6];
      weights[1] = sectorsizes[3] + sectorsizes[7];
      weights[2] = sectorsizes[1] + sectorsizes[2] + sectorsizes[8];
      weights[3] = sectorsizes[4] + sectorsizes[5] + sectorsizes[8];
      CreateInnerPoint(topo,topo.points[TPT_GN],points,weights);
    }
    else{
      // same as E1
      if (topo.cfg.type == FILL_EVEN){
        topo.points[TPT_GN] = topo.points[TPT_E1];
      }
      else{
        topo.points[TPT_GN] = topo.points[TPT_G2];
      }
    }

    #undef TPCOORD
  }
  
  // edges
  {
    topo.edges.SetCount(14 + 10);
    topo.numEdges = 0;

#define TPPOINT(what)   what

    // top __ edges
    AddEdge(topo,TPPOINT(TPT_E6),TPPOINT(TPT_CORNER0),fractions[1][0]);
    AddEdge(topo,TPPOINT(TPT_E6),TPPOINT(TPT_E5),topo.cfg.beta);
    AddEdge(topo,TPPOINT(TPT_CORNER3),TPPOINT(TPT_E5),fractions[1][1]);

    // top | edges
    AddEdge(topo,TPPOINT(TPT_CORNER0),TPPOINT(TPT_E0),fractions[0][0]);
    AddEdge(topo,TPPOINT(TPT_E6),TPPOINT(TPT_G1),fractions[0][0]);
    AddEdge(topo,TPPOINT(TPT_E5),TPPOINT(TPT_GP),fractions[0][0]);
    AddEdge(topo,TPPOINT(TPT_E7),TPPOINT(TPT_CORNER3),fractions[0][0]);
 
    // bottom | edges
    AddEdge(topo,TPPOINT(TPT_E2),TPPOINT(TPT_CORNER1),fractions[0][1]);
    AddEdge(topo,TPPOINT(TPT_E3),TPPOINT(TPT_G2),fractions[0][1]);
    AddEdge(topo,TPPOINT(TPT_E4),TPPOINT(TPT_GP),fractions[0][1]);
    AddEdge(topo,TPPOINT(TPT_CORNER2),TPPOINT(TPT_E7),fractions[0][1]);

    // bottom _ edges
    AddEdge(topo,TPPOINT(TPT_CORNER1),TPPOINT(TPT_E3),fractions[1][0]);
    AddEdge(topo,TPPOINT(TPT_E3),TPPOINT(TPT_E4),topo.cfg.alpha);
    AddEdge(topo,TPPOINT(TPT_CORNER2),TPPOINT(TPT_E4),fractions[1][1]);

    // specials

    // __
    AddEdge(topo,TPPOINT(TPT_E0),TPPOINT(TPT_G1),fractions[1][0]);
    // alpha beta
    if (topo.cfg.type != FILL_UNEVEN){

      // outer |
      AddEdge(topo,TPPOINT(TPT_E0),TPPOINT(TPT_E1),topo.cfg.alpha);
      AddEdge(topo,TPPOINT(TPT_E1),TPPOINT(TPT_E2),topo.cfg.beta);

      if (topo.cfg.beta){
        // alpha /
        AddEdge(topo,TPPOINT(TPT_G1),TPPOINT(TPT_GN),topo.cfg.alpha);

        // beta  \ 
        AddEdge(topo,TPPOINT(TPT_GN),TPPOINT(TPT_G2),topo.cfg.beta);
      }

      // __
      AddEdge(topo,TPPOINT(TPT_E1),TPPOINT(TPT_GN),fractions[1][0]);
    }
    else{
      // outer |
      AddEdge(topo,TPPOINT(TPT_E0),TPPOINT(TPT_E2),topo.cfg.summed);
      // inner |
      AddEdge(topo,TPPOINT(TPT_G1),TPPOINT(TPT_G2),topo.cfg.summed);
    }
    // __
    AddEdge(topo,TPPOINT(TPT_E2),TPPOINT(TPT_G2),fractions[1][0]);

    // alpha /
    AddEdge(topo,TPPOINT(TPT_GP),TPPOINT(TPT_G2),topo.cfg.alpha);

    // beta \ 
    AddEdge(topo,TPPOINT(TPT_G1),TPPOINT(TPT_GP),topo.cfg.beta);

    // last __
    AddEdge(topo,TPPOINT(TPT_GP),TPPOINT(TPT_E7),fractions[1][1]);

#undef TPPOINT

  }

  topo.sector.SetCount(9);

  switch(topo.cfg.type){
  case FILL_EVEN:
    {

      //////////////////////////////////////////////////
      //                   beta           
      //                                  
      //       C0________E6__E5_____C3    
      //  f00   |   0     | 1 |  2   |    
      //       E0________G1\  |      |    
      //  a     |   6   /    \GP_____E7   
      //  +    E1______GN  8 /|      |    
      //  b     |   7    \  / |  5   |    
      //       E2________G2   |      |    
      //  f01   |   3    | 4  |      |    
      //       C1________E3__E4______C2   
      //                                  
      //           f10    alpha f11       
      //                                  
      // triangle:                        
      //   sector 9 == 7+6                 

      // 9 sectors
      AddSector(topo, TPT_CORNER0,TPT_E0,TPT_G1,TPT_E6, sectorsizes[0]);
      AddSector(topo, TPT_E6,TPT_G1,TPT_GP,TPT_E5,      sectorsizes[1]);
      AddSector(topo, TPT_E5,TPT_GP,TPT_E7,TPT_CORNER3, sectorsizes[2]);
      AddSector(topo, TPT_E2,TPT_CORNER1,TPT_E3,TPT_G2, sectorsizes[3]);
      AddSector(topo, TPT_G2,TPT_E3,TPT_E4,TPT_GP,      sectorsizes[4]);
      AddSector(topo, TPT_GP,TPT_E4,TPT_CORNER2,TPT_E7, sectorsizes[5]);
      AddSector(topo, TPT_E0,TPT_E1,TPT_GN,TPT_G1,      sectorsizes[6]);
      AddSector(topo, TPT_E1,TPT_E2,TPT_G2,TPT_GN,      sectorsizes[7]);
      AddSector(topo, TPT_G1,TPT_GN,TPT_G2,TPT_GP,      sectorsizes[8]);
    }
    break;
  case FILL_UNEVEN:
    {
      //////////////////////////////////////////////////
      //                   beta
      //
      //       C0________E6__E5_____C3
      //  f00   |   0     | 1 |  2   |
      //       E0________G1\  |      |
      //  a     |        |   \GP_____E7
      //  +     |        | 8 /|      |
      //  b     |   9    |  / |  5   |
      //       E2________G2   |      |
      //  f01   |   3    | 4  |      |
      //       C1________E3__E4______C2
      //
      //           f10    alpha f11
      //
      // triangle:
      //   sector 9 == 7+6

      // 8 sectors
      AddSector(topo, TPT_CORNER0,TPT_E0,TPT_G1,TPT_E6, sectorsizes[0]);
      AddSector(topo, TPT_E6,TPT_G1,TPT_GP,TPT_E5,      sectorsizes[1]);
      AddSector(topo, TPT_E5,TPT_GP,TPT_E7,TPT_CORNER3, sectorsizes[2]);
      AddSector(topo, TPT_E2,TPT_CORNER1,TPT_E3,TPT_G2, sectorsizes[3]);
      AddSector(topo, TPT_G2,TPT_E3,TPT_E4,TPT_GP,      sectorsizes[4]);
      AddSector(topo, TPT_GP,TPT_E4,TPT_CORNER2,TPT_E7, sectorsizes[5]);
      AddSector(topo, TPT_E0,TPT_E2,TPT_G2,TPT_G1,      sectorsizes[9]);
      AddSector(topo, TPT_G1,TPT_G2,TPT_GP,-1,          sectorsizes[8]);
    }
    break;
  case FILL_EQUAL:
    {
      //////////////////////////////////////////////////
      //
      //
      //       C0____________E65____C3
      //  f00   |   0         |  2   |
      //       E0____________ |      |
      //  a     |   6        G1P_____E7
      //       E1_______    / |      |
      //       E2        GN2  |  5   |
      //        |        | 4  |      |
      //  f01   |   3    |    |      |
      //       C1________E3__E4______C2
      //
      //           f10    alpha f11
      //
      // triangle:
      //   sector 9 == 7+6

      // 6 sectors
      AddSector(topo, TPT_CORNER0,TPT_E0,TPT_G1,TPT_E6, sectorsizes[0]);
      AddSector(topo, TPT_E5,TPT_GP,TPT_E7,TPT_CORNER3, sectorsizes[2]);
      AddSector(topo, TPT_E2,TPT_CORNER1,TPT_E3,TPT_G2, sectorsizes[3]);
      AddSector(topo, TPT_G2,TPT_E3,TPT_E4,TPT_GP,      sectorsizes[4]);
      AddSector(topo, TPT_GP,TPT_E4,TPT_CORNER2,TPT_E7, sectorsizes[5]);
      AddSector(topo, TPT_E0,TPT_E1,TPT_GN,TPT_G1,      sectorsizes[6]);
    }
    break;
  }

}

void GridCap::CreateInnerPoint(Topo &topo, TPoint &tp, Point2 points[4], int weights[4])
{
  float sumweights = 0.0f;
  tp.onside = -1;
  tp.vert.coord = Point2(0.0f,0.0f);
  for (int i = 0; i < 4; i++){
    if (!weights[i]) continue;

    float weight = 1.0f / float(weights[i]);
    tp.vert.coord += points[i] * weight;
    sumweights += weight;
  }

  tp.vert.coord /= float(sumweights);
  tp.vert.vid = m_mesh->AppendNewVerts(1);
  m_mesh->P(tp.vert.vid) = Interpolate(topo,tp.vert.coord);
}

void GridCap::AddSector( Topo &topo, int a, int b, int c, int d, int size)
{
  if (!size)
    return;

  TEdge* edge0 = topo.GetEdge(a,b);
  TEdge* edge1 = topo.GetEdge(b,c);
  TEdge* edge2 = d > 0 ? topo.GetEdge(c,d) : topo.GetEdge(c,a);
  TEdge* edge3 = d > 0 ? topo.GetEdge(d,a) : NULL;

  SM_ASSERT(edge0 && edge1 && edge2 && (d < 0 || edge3));

  TSector &sector = topo.sector[topo.numSectors++];
  sector.numFaces = size;
  sector.numEdges = edge3 ? 4 : 3;
  sector.edges[0] = edge0;
  sector.edges[1] = edge1;
  sector.edges[2] = edge2;
  sector.edges[3] = edge3;

  sector.reversed[0] = false;
  sector.reversed[1] = false;
  sector.reversed[2] = false;
  sector.reversed[3] = false;

  if (*edge0->from == *edge1->from){
    sector.reversed[0] = true;
  }
  else if (*edge0->to == *edge1->to){
    sector.reversed[1] = true;
  }
  else if (*edge0->from == *edge1->to){
    sector.reversed[0] = true;
    sector.reversed[1] = true;
  }

  if ((!sector.reversed[1] && *edge1->to   == *edge2->to) ||
      ( sector.reversed[1] && *edge1->from == *edge2->to)) {
    sector.reversed[2] = true;
  }

  if (edge3){
    if ((!sector.reversed[2] && *edge2->to   == *edge3->to) ||
      (   sector.reversed[2] && *edge2->from == *edge3->to)) {
          sector.reversed[3] = true;
    }

    // initialize opposing fraccs
    // it's very likely we have one inner edge
    // for inner edges we inherit the outer edge fraccs and average them
    // an edge can have at max only 2 sectors

    int numOpposing = 0;
    int opposing[2];

    
    if (sector.edges[0]->size > 1 && (!sector.edges[0]->outer || !sector.edges[2]->outer)){
      opposing[numOpposing]= sector.edges[0]->fraccs.Count() ? 0 : 2;
      numOpposing++;
    }

    if (sector.edges[1]->size > 1 && (!sector.edges[1]->outer || !sector.edges[3]->outer)){
      opposing[numOpposing]= sector.edges[1]->fraccs.Count() ? 1 : 3;
      numOpposing++;
    }

    const bool* __restrict reversed = sector.reversed;

    for (int i = 0; i < numOpposing; i++){
      int from =  opposing[i];
      int to   = (from + 2) % 4;

      int between = sector.edges[to]->size - 1;
      if (sector.edges[to]->fraccs.Count() == 0){
        sector.edges[to]->fraccs.SetCount(between);
        for (int n = 0; n < between; n++){
          float newval = sector.edges[from]->GetFracc(n,!reversed[from]);
          sector.edges[to]->SetFracc(n,reversed[to],newval);
        }
      }
      else {
        for (int n = 0; n < between; n++){
          float oldval = sector.edges[to]->GetFracc(n,reversed[to]);
          float newval = sector.edges[from]->GetFracc(n,!reversed[from]);
          float avg = (oldval + newval) * 0.5f;
          sector.edges[to]->SetFracc(n,reversed[to],avg);
        }
      }
    }
  }
}


void GridCap::MakeSidePoint( Topo &topo, TPoint &tp, int side, int idx)
{
  bool rev = m_layoutReverse;
  const GridLayout::Side& layoutside   = m_layout->GetSide(topo.sides[side].layoutSide);
  int oidx = (rev ? layoutside.size - idx : idx);  
  const GridLayout::BorderPoint &opoint = m_layout->GetBorderPoint(layoutside.outstart + oidx);
  tp.onside  = side;
  tp.sideidx = idx;
  tp.vert.vid = opoint.vid;

  SM_ASSERT(-0.01f < opoint.fracc && opoint.fracc < 1.01f);

  int layoutSide = topo.sides[side].layoutSide;
  if (oidx >= layoutside.size){
      layoutSide = (layoutSide + 1) % 4;
  }

  switch (layoutSide)
  {
  case 0:
    tp.vert.coord   = Point2(0.0f,opoint.fracc);
    break;
  case 1:
    tp.vert.coord   = Point2(opoint.fracc,1.0f);
    break;
  case 2:
    tp.vert.coord   = Point2(1.0f,1.0f-opoint.fracc);
    break;
  case 3:
    tp.vert.coord   = Point2(1.0f-opoint.fracc,0.0f);
    break;
  }
}


void GridCap::AddEdge( Topo &topo, int a, int b, int size )
{
  TPoint* from = &topo.points[a];
  TPoint* to   = &topo.points[b];
  if (!size || from->vert.vid == to->vert.vid || topo.GetEdge(a,b))
    return;

  TEdge& edge = topo.edges[topo.numEdges++];
  edge.from   = from;
  edge.to     = to;
  edge.size   = size;
  edge.outer  = false;

  edge.verts.Init();
  edge.fraccs.Init();

  if (from->isOnSide() && to->isOnSide() && abs(from->onside - to->onside) != 2){
    edge.outer = true;
    edge.verts.SetCount(size+1);
    edge.verts[0]    = from->vert;
    edge.verts[size] = to->vert;

    int numBetween = size-1;
    int usedside = from->onside;
    int sideidx  = from->sideidx;
    int sign     = 1;
    int delta    = 1;

    if (numBetween){
      edge.fraccs.SetCount(size-1);
      TPoint tp;
      MakeSidePoint(topo,tp,usedside, sideidx + (numBetween*sign) + delta);
      if (tp.vert.vid != to->vert.vid){
        usedside = to->onside;
        sideidx  = to->sideidx;
        delta    =  numBetween;
        sign     = -1;
      }

      MakeSidePoint(topo,tp,usedside, sideidx + (-1*sign) + delta);
      if (tp.vert.vid != from->vert.vid){
        usedside = to->onside;
        sideidx  = to->sideidx;
        delta    =  -numBetween;
        sign     =  1;
      }
#if 0
      MakeSidePoint(topo,tp,usedside, sideidx + (-1*sign) + delta);
      if (tp.vert.vid != from->vert.vid){
        sign = sign; // ERROR
      }
#endif
    }

    float edgeLength = (from->vert.coord - to->vert.coord).Length();

    for (int v = 0; v < numBetween; v++){
      TPoint tp;
      MakeSidePoint(topo,tp,usedside, sideidx + (v*sign) + delta);
      float dist = (from->vert.coord - tp.vert.coord).Length()/edgeLength;
      edge.verts[v + 1] = tp.vert;
      edge.fraccs[v] = dist;
    }
  }
}


void GridCap::Merge( Topo &topo, const Topo &upper, const Topo &lower  )
{

}

Point3 GridCap::Interpolate( Topo &topo, const Point2& coord )
{
  SM_ASSERT(topo.sides.Count() == 4);

  // x 0 to 2
  // y 1 to 3

  //////////////////////// 
  //  side   q:3
  //     0,0______
  //       |      |
  //  m:0  |      |  n:2
  //       |______|
  //
  //         p:1

  Point3 pfromouth;
  Point3 pfromh;
  Point3 ptoh;
  Point3 ptoouth;

  Point3 pfromoutv;
  Point3 pfromv;
  Point3 ptov;
  Point3 ptooutv;

  m_layout->InterpolateSide(0,      coord.y, pfromh, pfromouth);
  m_layout->InterpolateSide(2, 1.0f-coord.y, ptoh,   ptoouth);

  m_layout->InterpolateSide(1,      coord.x, ptov,   ptooutv);
  m_layout->InterpolateSide(3, 1.0f-coord.x, pfromv, pfromoutv);

  Point3 pos =  m_gip->Run(coord, pfromouth, pfromh, ptoh, ptoouth, pfromoutv, pfromv, ptov, ptooutv );

  return pos;
}

void GridCap::CreateEdgeVertices( Topo &topo, TEdge &edge )
{
  if (edge.verts.Count() == edge.size + 1)
    return;

  int numBetween   = edge.size-1;
  int vidstart = numBetween ? m_mesh->AppendNewVerts(numBetween) : 0;
  edge.verts.SetCount(edge.size + 1);
  edge.verts[0]         = edge.from->vert;
  edge.verts[edge.size] = edge.to->vert;

  for (int v = 0; v < numBetween; v++){
    Point2 coord = LERP(edge.from->vert.coord, edge.to->vert.coord, edge.fraccs[v]);
    int    vid   = vidstart + v;

    m_mesh->P(vid) = Interpolate(topo, coord);

    edge.verts[v + 1].vid   = vid;
    edge.verts[v + 1].coord = coord;
  }
}

void reversedLayoutVertices(std::vector<int> &verts, int w, int h){
  std::vector<int> reordered;
  reordered.resize(w * h);

  int i = 0;
  for (int y = 0; y < h; y++){
    for (int x = w-1; x >= 0; x--, i++){
      reordered[i] = verts[y*w + x];
    }
  }
  verts = reordered;
  //std::reverse(verts.begin(),verts.end()); 
}

static inline void appendFaces(Tab<int> *newFaces, int first, int count)
{
  if (newFaces && first >= 0){
    int oldCount = newFaces->Count();
    newFaces->SetCount( oldCount + count );
    int* start = newFaces->Addr(oldCount);
    for (int i = 0; i < count; i++){
      start[i] = first + i;
    }
  }
}

void GridCap::CreateSectorMesh( Topo &topo, TSector &sector, Tab<int> *newFaces )
{
  // for all sectors create inner vertices and quads
  if (sector.numEdges == 3) {

    //////////////////////// 
    //  side 
    //     0,0
    //       |\  b:2
    //  c:0  | \ 
    //       | / 
    //       |/  a:1
    //

    SM_ASSERT(  sector.edges[0]->size == sector.edges[1]->size + sector.edges[2]->size - 1);
    SM_ASSERT(  sector.edges[1]->size >= sector.edges[2]->size );
    
    // create 2 quad rectangles: upper/lower
    // triangle position is determined by a & b sizes

    //////////////////////////
    //                         
    //                         
    // c 4     .               
    //          |\  b 2        
    //   |a|-1 .|U\            
    //          |\U\.          
    //         .|U\/\          
    //      tri |\/\/.         
    //         .|/\/.          
    //   |b|-1  |L/.          
    //          |/  a 3        
    //         .               
    //                         

    int tripos = sector.edges[1]->size - 1;

    int lowerh = sector.edges[2]->size - 1 + 1;
    int lowerw = sector.edges[1]->size     + 1;

    int upperh = 1 + 1;
    int upperw = sector.edges[1]->size - 1 + 1;

    const bool* __restrict reversed = sector.reversed;
    int vidnew = -1;
    int newverts = (lowerw-2) * (lowerh-1);
    if (newverts > 0){
      // first new vertex guaranteed to be triangle tip
      vidnew = m_mesh->AppendNewVerts( newverts );
    }
    else{
      // triangle tip is either on side a or b
      if (tripos == 0){
        vidnew = sector.edges[2]->GetVert(1, !reversed[2]).vid;
      }
      else{
        vidnew = sector.edges[1]->GetVert(1,  reversed[1]).vid;
      }
    }
    
    // create new verts (downwards from triangle)
    int vidrunner = vidnew;
    for (int y = 0; y < (lowerh-1); y++){
      const TVert& fromY  = sector.edges[0]->GetVert(tripos+1+y,  reversed[0]);
      const TVert& toY    = sector.edges[2]->GetVert(1+y,        !reversed[2]);

      for (int x = 0; x < (lowerw-2); x++){
        const TVert& fromX = sector.edges[0]->GetVert(tripos-x,   reversed[0]);
        const TVert& toX   = sector.edges[1]->GetVert(1+x,        reversed[1]);

        Point2 coord;
        Line2DIntersect(fromX.coord, toX.coord, fromY.coord, toY.coord, coord);
        coord = LERP(fromX.coord,toX.coord,coord.x);

        m_mesh->P(vidrunner) = Interpolate(topo, coord);

        vidrunner++;
      }
    }

    // create lower and upper quad rectangles
    std::vector<int> verts;
    if (upperw * upperh > 1){
      // upper half
      verts.clear();
      verts.resize(upperw * upperh);
      int idx = 0;
      for (int x = 0; x < upperw; x++){
          // first row guaranteed on c
          verts[idx++] = sector.edges[0]->GetVert(tripos-x,  reversed[0]).vid;
      }
      // second row ends at b but inbetweens could be on vidnew or a
      if (newverts > 0){
        for (int x = 0; x < upperw-1; x++){
          verts[idx++] = vidnew + x;
        }
      }
      else{
        for (int x = 0; x < upperw-1; x++){
          verts[idx++] = sector.edges[1]->GetVert(1+x, reversed[1]).vid;
        }
      }
          verts[idx++] = sector.edges[2]->GetVert(1,  !reversed[2]).vid;


      if (m_layoutReverse){
        reversedLayoutVertices(verts,upperw,upperh);
      }
      int facecnt;
      int first = MNMesh_fillQuadRectangle(m_mesh,&verts[0],upperw, upperh, facecnt);
      appendFaces(newFaces,first,facecnt);
    }

    if (lowerw * lowerh > 1){
      // lower half
      verts.clear();
      verts.resize(lowerw * lowerh);

      // first row on vidnew
      int idx = 0;
      vidrunner = vidnew;
      for (int y = 0; y < lowerh-1; y++){
        // first on C
          verts[idx++] = sector.edges[0]->GetVert(tripos+1+y, reversed[0]).vid;
        // inbetweens
        for (int x = 1; x < lowerw-1; x++,vidrunner++){
          verts[idx++] = vidrunner;
        }
        // last on B
          verts[idx++] = sector.edges[2]->GetVert(1+y, !reversed[2]).vid;
      }

      // last row on a
      for (int x = 0; x < lowerw; x++){
        verts[idx++] = sector.edges[1]->GetVert(x, reversed[1]).vid;
      }

      if (m_layoutReverse){
        reversedLayoutVertices(verts,lowerw, lowerh);
      }

      int facecnt;
      int first = MNMesh_fillQuadRectangle(m_mesh,&verts[0],lowerw, lowerh, facecnt);
      appendFaces(newFaces,first,facecnt);
    }

    // create triangle
    {
      int triverts[3] = {
        sector.edges[0]->GetVert(tripos,    reversed[0]).vid,
        sector.edges[0]->GetVert(tripos+1,  reversed[0]).vid,
        vidnew,
      };

      if (m_layoutReverse){
        int tmp = triverts[0];
        triverts[0] = triverts[2];
        triverts[2] = tmp;
      }

      int first = m_mesh->CreateFace(3,triverts);
      appendFaces(newFaces,first,1);
    }


  }
  else if (sector.numEdges == 4){

    //////////////////////// 
    //  side   d:3
    //     0,0______
    //       |      |
    //  a:0  |      |  c:2
    //       |______|
    //
    //         b:1

    SM_ASSERT(  sector.edges[0]->size == sector.edges[2]->size &&
                sector.edges[1]->size == sector.edges[3]->size);

    int h = sector.edges[0]->size + 1;
    int w = sector.edges[1]->size + 1;

    const bool* __restrict reversed = sector.reversed;
    std::vector<int> verts;
    verts.resize(w * h);

    int newverts = (w - 2) * (h - 2);
    int vidnew = -1;
    if (newverts > 0){
      vidnew = m_mesh->AppendNewVerts( newverts );
    }

    int pos = 0;
    for (int x = 0; x < w; x++){
      verts[pos++] = sector.edges[3]->GetVert(x, !reversed[3]).vid;
    }
    for (int y = 1; y < h-1; y++){
      const TVert& fromY  = sector.edges[0]->GetVert(y,  reversed[0]);
      const TVert& toY    = sector.edges[2]->GetVert(y, !reversed[2]);

      verts[pos++] = fromY.vid;
      for (int x = 1; x < w-1; x++){
        verts[pos++] = vidnew;
        const TVert& fromX = sector.edges[3]->GetVert(x, !reversed[3]);
        const TVert& toX   = sector.edges[1]->GetVert(x,  reversed[1]);
        
        Point2 coord;
        Line2DIntersect(fromX.coord, toX.coord, fromY.coord, toY.coord, coord);
        coord = LERP(fromX.coord,toX.coord,coord.x);
        
        m_mesh->P(vidnew) = Interpolate(topo, coord);

        vidnew++;
      }
      verts[pos++] = toY.vid;
    }
    for (int x = 0; x < w; x++){
      verts[pos++] = sector.edges[1]->GetVert(x, reversed[1]).vid;
    }

    if (m_layoutReverse){
      reversedLayoutVertices(verts,w,h);
    }

    int facecnt;
    int first = MNMesh_fillQuadRectangle(m_mesh,&verts[0],w,h,facecnt);
    appendFaces(newFaces,first,facecnt);
  }
  else {
    SM_ASSERT(0);
  }
}

void GridCap::CreateMesh( Topo &topo, Tab<int> *newFaces )
{
  for (int i = 0; i < topo.numEdges; i++)
  {
    TEdge &edge = topo.edges[i];
    CreateEdgeVertices(topo,edge);
  }
  for (int i = 0; i < topo.numSectors; i++ )
  {
    TSector &sector = topo.sector[i];
    CreateSectorMesh(topo,sector,newFaces);
  }
}


//////////////////////////////////////////////////////////////////////////
// Main Func


BOOL  PolyObject_GridFill(PolyObject *pobj, const SMInterpolation& interpolation, const GridCapConfig &cfg, int edge)
{
  static GridInterpolatorLinear       theGridInterpolatorLinear;
  static GridInterpolatorCatmullRom   theGridInterpolatorCatmullRom;
  static GridInterpolatorBezier       theGridInterpolatorBezier;
  static GridInterpolator*      theGridInterpolators[SM_INTERPOLATORS];
  theGridInterpolators[GridInterpolatorLinear::TYPE]      = &theGridInterpolatorLinear;
  theGridInterpolators[GridInterpolatorCatmullRom::TYPE]  = &theGridInterpolatorCatmullRom;
  theGridInterpolators[GridInterpolatorBezier::TYPE]      = &theGridInterpolatorBezier;
 
  GridInterpolator* gip = theGridInterpolators[interpolation.type];
  gip->m_weight     = interpolation.weight;
  gip->m_tangentweight  = interpolation.tangentweight;
  gip->m_inner = interpolation.inner;
  gip->Init();

  // get borders
  Tab<int>  edgeloop;
  Tab<int>* loop = NULL;
  MNMesh  *mesh = &pobj->mm;
  MNMeshBorder borders;
  int origcnt = 0;

  if (edge >= 0){
    MNMeshUtilities util(mesh);
    loop = util.GetBorderFromEdge(edge,edgeloop) ? &edgeloop : NULL;
  }
  else{
    mesh->GetBorder(borders,MNM_SL_EDGE);

    for (int i = 0; i < borders.Num(); i++){
      if (!borders.LoopTarg(i) && edge < 0) continue;
      origcnt++;
    }
  }
  
  if (loop){
    origcnt = 1;
  }
  else if (edge >= 0){
    // didn't find anything
    return FALSE;
  }


  theHold.Begin();

  TopoChangeRestore *restore =  new TopoChangeRestore(pobj);
  if (restore){
    restore->Before();
  }
  
  BOOL dosmooth = FALSE;
  BOOL retry = TRUE;
  int fillcnt = 0;
  Tab<int>  newCapFaces;

  while(retry){
    retry = FALSE;

    // one border a time
    // as topology changes

    int bordercnt = loop ? 1 : borders.Num();
    for (int i = 0; i < bordercnt; i++){
      if (!loop && !borders.LoopTarg(i)) continue;

      Tab<int>  *curloop = loop ? loop : borders.Loop(i);

      GridLayout layout;
      if (layout.Init(mesh,curloop))
      {
        GridFill gridFill;
        gridFill.Run(&layout,gip);
        fillcnt++;
      }
      else if (GridCap::ValidLayout(layout))
      {
        GridCap  gridCap;
        gridCap.Run(&layout,gip,cfg,&newCapFaces);
        fillcnt++;
        dosmooth = TRUE;
      }
    }

    // topology change means we need to refetch
    // borders
    if (!loop){
      borders.Clear();
      mesh->GetBorder(borders,MNM_SL_EDGE);
    }
  }


  // ask if regular cap should be used
  bool changed = fillcnt > 0;

  if (origcnt > fillcnt && (MessageBoxA(NULL,"The topology could not be properly filled with quads. This typically happens when the rectangle region created from the border has too different edge counts on opposing sides.\r\nDo you want to apply a regular cap instead?","Alles Im Fluss - Modelling Toolkit",MB_YESNO | MB_DEFBUTTON1) == IDYES)){
    
    if (loop){
      mesh->ClearEFlags(MN_USER);
      mesh->E(edge)->SetFlag(MN_USER);
      mesh->GetBorder(borders,MNM_SL_EDGE,MN_USER);
    }
    mesh->FillInBorders(&borders);
    if (loop){
      mesh->E(edge)->SetFlag(MN_USER,false);
    }
    changed = true;
  }

  //mesh->FillInMesh();
  if (newCapFaces.Count() && dosmooth){
    BitArray  selFaces( mesh->numf );
    for (int i = 0; i < newCapFaces.Count(); i++){
      selFaces.Set(newCapFaces[i]);
    }
    MNMeshSurfaceRelax relax(pobj->GetMesh(),selFaces);
    SMRelax settings;
    settings.alpha = 0.75;
    settings.weight = 1.0;
    settings.relaxIters = 50;
    settings.relaxAmount = 0.5;

    relax.Run(settings);
  }

  // undo
  if (restore){
    restore->After();
    theHold.Put(restore);

    if (changed){
      theHold.Accept(_M("AIF Fill"));
    }
    else{
      theHold.Cancel();
    }
  }

  return fillcnt == origcnt;
}


#endif


