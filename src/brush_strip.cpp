/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#include "coretool.h"
#include "brush_strip.h"
#include <meshadj.h>
#if MAX_RELEASE >= 14000
#include <Graphics/IDisplayManager.h>
#endif

#ifdef SM_SUPPORT_STRIP

/*
  Start Stroke:
    # from open edge
      # from sel connected
        +delta edge hops to reference

      + vertexdeltas to reference
      + check for "pull"/"face" mode
        (depending on view angle along open edge)
      + plane dist to reference
    # open space
      + "face" mode
      + plane dist from workplane/projobj


  - fn ConstrainStrokePoint
    # hug when open vert connect is possible

      + adjust strokepoint to compensate
        hug snap distance (varying stride)

      + adjust stroke plane "distance" to hugvert distance
        or average if both hug
      + flag hugged connect verts

  Proc Stroke
    # hit with workplane (or projobj)

    # Advance/Fill Strokes (fast mouse movement, or projected distances)
      + interpolate points & ConstrainStrokePoint
        (one after another, cause of constrain deltas)


    # Preview
      + half rotate last verts
      + "open" verts rotate/scale/hug fully with current brush


  - fn ReflowStroke
    + interpolate distances for start-hug, hug-hug, hug-end, start-end
    + do not touch "hugged" strokes, basically only "free"


  End Stroke
    # on open edge if started open
      - fn ReStroke
      + find corresponding verts
      + reflow stroke
      + rebuild mesh (keep hugs)
      + attach

    # open space
      + attach

    # selected other unconnected open edges
      # end connected
        + find corresponding open

      + replicate & transform stroke
      + full ReStroke for each


*/



void SMBrushStrip::Init(SketchModeler *oSM)
{
  m_SM = oSM;

  m_strokeproc.Init(m_SM->GetMConVert());
  m_strokeprocTemp.Init(m_SM->GetMConVert());
}
void SMBrushStrip::Display(TimeValue t, ViewExp *vpt, int flags)
{
  if (!m_SM->GetSafeNode() || m_SM->WasMeshChanged())
    return;

  Matrix3 objmat = m_SM->m_node->GetObjectTM(m_SM->m_ip->GetTime());
  GraphicsWindow *gw = vpt->getGW();
  gw->setTransform(objmat);
  DWORD savedLimits = gw->getRndLimits();
  Point3 rp[3];

  if (m_stroking && 
    ( m_type == TYPE_FREE || 
      m_type == TYPE_EXTRUDE || 
      m_type == TYPE_EXTEND || 
      m_type == TYPE_REPLAYFREE ||
      m_type == TYPE_REPLAYEXTEND ||
      m_type == TYPE_REPLAYEXTRUDE) 
      
      && m_pmesh.FNum())
  {
    MNMesh *pMeshToDisplay = m_drawconnected ? &m_pmeshTemp : &m_pmesh;
    Material* pMaterial = &m_SM->m_hwmat;

  #ifdef MESH_CAGE_BACKFACE_CULLING
    if (savedLimits & GW_BACKCULL) pMeshToDisplay->UpdateBackfacing (gw, true);
  #endif

    gw->setRndLimits((savedLimits & ~GW_TEXTURE) | GW_Z_BUFFER );

    //pMeshToDisplay->updateRVerts(gw);
    //pMeshToDisplay->InvalidateHardwareMesh();

#if MAX_RELEASE >= 14000
    if (1 || MaxSDK::Graphics::IsRetainedModeEnabled())
    {
      // manually draw for now
      gw->setMaterial(*pMaterial);
      Point3 xyz[4];
      Point3 nor[4];
      Point3 uvw[4];
      uvw[0] = Point3(0.0f,0.0f,0.0f);
      uvw[1] = Point3(1.0f,0.0f,0.0f);
      uvw[2] = Point3(0.0f,1.0f,0.0f);

      Tab<int> tri;

      gw->startTriangles();
      for(int i=0;i<pMeshToDisplay->numf;++i)
      {
        MNFace *face = &pMeshToDisplay->f[i];
        if (face->GetFlag (MN_DEAD)) continue;

        tri.ZeroCount();
        face->GetTriangles(tri);
        int		*vv		= face->vtx;
        int		deg		= face->deg;
        DWORD	smGroup = face->smGroup;
        for (int tt=0; tt<tri.Count(); tt+=3)
        {
          int *triv = tri.Addr(tt);
          for (int z=0; z<3; z++) xyz[z] = pMeshToDisplay->v[vv[triv[z]]].p;
          nor[0] = nor[1] = nor[2] = pMeshToDisplay->GetFaceNormal(i, TRUE);
          gw->triangleN(xyz,nor,uvw);
        }
      }
      gw->endTriangles();
    }
    else
#endif
    {
      pMeshToDisplay->render(gw, pMaterial,NULL,COMP_ALL,1);
    }

    {
      gw->setRndLimits((savedLimits & ~GW_ILLUM) | GW_ALL_EDGES | GW_Z_BUFFER);

      // We need to draw a "gizmo" version of the polymesh:
      //Point3 colSel=GetSubSelColor();
      //Point3 colTicks=GetUIColor (COLOR_VERT_TICKS);
      gw->setColor (LINE_COLOR, pMaterial->Kd * 0.25f);
      int i;
      int es[3];
      bool edgeLev = false;
      bool faceLev = true;

      gw->startSegments();
      for (i=0; i<pMeshToDisplay->nume; i++) {
        MNEdge *pEdge = pMeshToDisplay->E(i);
        if (pEdge->GetFlag (MN_DEAD)) continue;
        if (pMeshToDisplay->f[pEdge->f1].GetFlag (MN_HIDDEN) &&
          ((pEdge->f2 < 0) || pMeshToDisplay->f[pEdge->f2].GetFlag (MN_HIDDEN))) continue;
    #ifdef MESH_CAGE_BACKFACE_CULLING
        if ((savedLimits & GW_BACKCULL) && pEdge->GetFlag (MN_BACKFACING)) continue;
    #endif
        if (pEdge->GetFlag (MN_EDGE_INVIS)) {
          if (!edgeLev) continue;
          es[0] = GW_EDGE_INVIS;
        } else {
          es[0] = GW_EDGE_VIS;
        }

        rp[0] = pMeshToDisplay->v[pEdge->v1].p;
        rp[1] = pMeshToDisplay->v[pEdge->v2].p;
        if (es[0] == GW_EDGE_VIS)
        {
          gw->segment(rp,1);
        }
      }
      gw->endSegments();
    }
  }

  if (m_drawpointsforce || (m_drawpoints && (m_stroking && 
    ( m_type == TYPE_FREE || 
      m_type == TYPE_EXTRUDE || 
      m_type == TYPE_EXTEND || 
      m_type == TYPE_REPLAYFREE ||
      m_type == TYPE_REPLAYEXTEND ||
      m_type == TYPE_REPLAYEXTRUDE) && m_pmesh.FNum())))
  {
    gw->setRndLimits(savedLimits & ~(GW_ILLUM | GW_Z_BUFFER));
    Color color = m_SM->m_hwmat.Kd * 0.25f;

    if (m_drawpointsforce == 1){
      color = m_SM->GetDisplayCallback()->m_hitcolor;
    }
    gw->setColor (LINE_COLOR, color);

    gw->startSegments();
    int lastpt;
    const StrokeProcessor& proc = m_drawpointsforce ? m_strokeprocTemp : m_strokeproc;
    const StrokePointArray& pts = proc.GetStrokePointArray(lastpt);
    for (int i = 0; i < lastpt; i++){
      rp[0] = pts[i].opos;
      rp[1] = pts[i+1].opos;
      gw->segment(rp,1);
    }

    gw->endSegments();

    if (m_drawpointsforce){
      color= m_SM->GetDisplayCallback()->m_markercolor;
      gw->setColor (LINE_COLOR, color);
      gw->marker(&m_highlightanchor, DOT4_MRKR);
      gw->marker(&m_highlightanchor, DOT2_MRKR);
    }
  }


  if (m_drawhighlights){
    gw->setRndLimits(savedLimits & ~(GW_ILLUM | GW_Z_BUFFER));
    Color color =  m_SM->GetDisplayCallback()->m_hitcolor;
    gw->setColor (LINE_COLOR,color);

    int edgecnt = m_highlightedges.Count();
    for (int i = 0; i < edgecnt; i++){
      MNEdge  *edge = m_SM->m_mesh->E(m_highlightedges[i]);
      MNMesh_renderMNEdgeThick(gw,m_SM->m_mesh,edge);
    }
    color= m_SM->GetDisplayCallback()->m_markercolor;
    gw->setColor (LINE_COLOR, color);
    gw->marker(&m_highlightanchor, DOT4_MRKR);
    gw->marker(&m_highlightanchor, DOT2_MRKR);
  }

  gw->setRndLimits(savedLimits);



}
void SMBrushStrip::GetViewportRect( TimeValue t, ViewExp *vpt, Rect *rect )
{

  // best is strip brush does full redraws
  *rect += IPoint2(0,0);
  *rect += IPoint2(vpt->getGW()->getWinSizeX(),vpt->getGW()->getWinSizeY());
}

void SMBrushStrip::SetConstrained(BOOL state)
{
  if (state == m_SM->m_workplane.IsConstrained()) return;

  if (state){
    if (m_stroking){
      if (m_delaystart || !m_strokeproc.HasPoints()){
        m_delayconstr = TRUE;
        return;
      }
      Point3 wpos = m_SM->m_node->GetObjectTM(m_SM->m_ip->GetTime()).PointTransform(
        ( m_type == TYPE_REPLAYFREE ||
          m_type == TYPE_REPLAYEXTEND ||
          m_type == TYPE_REPLAYEXTRUDE) ?  
          m_strokeproc.GetFirstFixedPoint() :
          m_strokeproc.GetLastFixedPoint()
        );
      m_SM->m_workplane.EnableConstrained(wpos);
      m_delayconstr = FALSE;
    }
    else{
      m_delayconstr = TRUE;
    }
  }
  else{
    m_SM->m_workplane.DisableConstrained();
    m_delayconstr = FALSE;
  }

}

void SMBrushStrip::SetAlternative(BOOL state)
{

}

int SMBrushStrip::GetEndEdge(const SMHit &hit,bool discardfirst)
{
  if (SMHit_hasNoFaceHit(hit) || !MNMesh_isFaceOpen(m_SM->m_mesh,hit.face.fid))
    return -1;
#if 0
  Matrix3 objmat = m_SM->m_node->GetObjectTM(m_SM->m_ip->GetTime());
  Matrix3 objmatinv = Inverse(objmat);

  Point3  normal = hit.face.normal;
  float dist = DotProd(hit.face.normal,hit.face.pos);
  Point3  laststrokepos = m_strokeproc.GetLastFixedPoint();
  Point3  projpos;
  Ray   ray;

  Point3  viewdir;
  Point3  viewpos;
  BOOL viewortho = m_SM->m_workplane.GetViewParams(viewdir,viewpos);

  viewdir = objmatinv.VectorTransform(viewdir);
  viewpos = objmatinv.PointTransform(viewpos);

  if (viewortho){
    ray.dir = viewdir;
    ray.p = laststrokepos - (viewdir*(viewpos-laststrokepos).Length());
  }
  else{
    ray.p = viewpos;
    ray.dir = (laststrokepos-viewpos).Normalize();
  }

  PlaneIntersect(ray,normal,dist,projpos);
#else
  Point3 projpos = hit.face.pos;
#endif
  int edge = MNMesh_findClosestEdge(m_SM->m_mesh,hit.face.fid,projpos,discardfirst ? m_startedge : -1,true);
  //edge = MNMesh_isEdgeOpen(SM->m_mesh,edge) ? edge : -1;

  return edge;
}



void SMBrushStrip::Proc(ViewExp *vpt, const SMHit &hit, IPoint2 &m, IPoint2 &lastm)
{
  m_SM->m_fullredraw = TRUE;

  INode *node = m_SM->GetSafeNode();
  if (!node || m_refine)
    return;

  Matrix3 objmat = node->GetObjectTM(m_SM->m_ip->GetTime());
  Matrix3 objmatinv = Inverse(objmat);
  MNMesh  *mesh = m_SM->m_mesh;
  int solevel = m_SM->m_ip->GetSubObjectLevel();

  // update workplane
  if (!m_stroking || (m_delaystart && m_startface < 0)){
    if (!m_SM->m_workplane.IsLocal()){
      m_SM->m_workplane.SetPoint(SMHit_hasNoFaceHit(hit) ? objmat.GetRow(3) : objmat.PointTransform(hit.face.pos));
    }
  }

  if (m_delayconstr){
    SetConstrained(TRUE);
  }

  // hit test with workplane
  Ray curray = hit.wray;
  Point3 curposworld;
  BOOL collhit;

  if (m_type == TYPE_INSET){
    if (!m_SM->m_workplane.Intersect(curray,curposworld,true))
      return;
    collhit = FALSE;
  }
  else{
    if (!m_SM->m_workplane.CorrectIntersect(m_SM->m_workplane.Intersect(curray,curposworld,true),curray,curposworld,collhit))
      return;
  }

  float constrscale = m_SM->m_workplane.GetLastConstrainScale();
  Point3 objpos = objmatinv.PointTransform(curposworld);
  //float cursize = LERP(hit.brushinner,hit.brush,hit.pressure);
  float cursize = hit.brush;

  // edge preview
  int closestedge = -1;
  int closestface = -1;

  m_drawhighlights  = FALSE;
  m_drawconnected   = FALSE;
  m_drawpointsforce = FALSE;

  if (m_type == TYPE_EXTEND || 
      m_type == TYPE_FREE || 
      m_type == TYPE_REPLAYFREE || 
      m_type == TYPE_REPLAYEXTEND)
  {
    Mode facetype = TYPE_IGNORE;


    if (m_stroking && m_delaystart && m_startface >= 0)
    { // delay start only for non-extrude

      // we check for "from edge"
      // intersect with face plane
      curray.p = objmatinv.PointTransform(curray.p);
      curray.dir = objmatinv.VectorTransform(curray.dir);
      PlaneIntersect(curray,m_startnormal,m_startdist,objpos);
      closestedge = MNMesh_findClosestEdge(mesh,m_startface,objpos,-1,true);
      //closestedge = MNMesh_isEdgeOpen(mesh,closestedge) ? closestedge : -1;
      closestface = solevel == SM_SO_FACE ? m_startface : -1;
    }
    else if (!m_stroking && SMHit_hasFaceHit(hit))
    { // we are just about to start
      int origclosest = MNMesh_findClosestEdge(mesh,hit.face.fid,hit.face.pos,-1,true);
      closestedge = origclosest;
      closestedge = MNMesh_isEdgeOpen(mesh,closestedge) ? closestedge : -1;
#ifdef SM_SUPPORT_STRIP_FACEOPS
      if (solevel == SM_SO_FACE && (MNMesh_isFaceSelected(mesh,hit.face.fid) || 
                                    MNMesh_isFaceSelected(mesh,mesh->E(hit.subobj.eid)->OtherFace(hit.face.fid))))
      {
        Matrix3 facematrix;
        closestface = MNMesh_isFaceSelected(mesh,hit.face.fid) ? hit.face.fid : mesh->E(hit.subobj.eid)->OtherFace(hit.face.fid);
        // if selected and extrusion possible

        facetype = GetFaceType(mesh,closestface,objmatinv,facematrix);
        if (closestface >= 0 && (facetype == TYPE_EXTRUDE || facetype == TYPE_GRABPATH))
        {
          closestedge = hit.subobj.eid;
        }
        else{
          // we would clone
          closestedge = -1;
          closestface = -1;
        }
      }
#endif
    }

    if ((closestedge >= 0 || closestface >= 0) && (facetype == TYPE_EXTRUDE || facetype == TYPE_IGNORE)){
      Tab <int> verts;
      int seededge = m_strokeproc.GetStartEdges(closestedge,closestface,hit.face.pos,cursize,m_highlightedges,verts,mesh);

      if (seededge >= 0){
        m_highlightanchor = closestface >= 0 ? hit.face.pos : MNMesh_closestPointOnEdge(mesh,seededge,hit.face.pos);
        m_drawhighlights = TRUE;
      }
      else{
        m_drawhighlights = FALSE;
      }
    }

    if (closestedge >= 0 && closestface >= 0 && facetype == TYPE_GRABPATH){
      m_drawpointsforce = m_strokeprocTemp.ExtractReplay(closestedge,closestface,mesh,&m_SM->m_workplane,objmat );
      if (m_drawpointsforce){
        m_highlightanchor = m_strokeprocTemp.GetFirstFixedPoint();
      }
    }
  }

  if (!m_stroking){
    if (collhit && m_SM->m_workplane.HasCollAlignment()){
      Matrix3 wmat = m_SM->m_workplane.GetMatrix();
      Point3 normal;
      if (m_SM->m_workplane.CorrectNormalVisible(curposworld,normal)){
        wmat = Matrix3_orientZ(wmat,normal);
      }
      m_SM->SetCirclePts(wmat,curposworld,cursize,hit.brush);
    }
    else{
      m_SM->SetCirclePts(m_SM->m_workplane.GetMatrix(),curposworld,cursize,hit.brush);
    }

    m_SM->m_node->NotifyDependents(FOREVER,PART_TM,REFMSG_CHANGE);
    return;
  }

  if (m_delaystart){
    if (m_startface >= 0){
      if( hit.face.fid == m_startface){ // solevel == SM_SO_FACE &&
        return;
      }

      if (closestedge < 0 && !MNMesh_isEdgeOpen(mesh,m_startedge))
      { // start normal
        m_startface = -1;
        m_startedge = -1;
        m_strokeproc.Start(objpos,-1,-1,cursize);
        m_delaystart = FALSE;

        m_type = TYPE_FREE;
      }
      else
      { // start with openedge

        m_startedge = closestedge < 0 ? m_startedge : closestedge;
        objpos = MNMesh_closestPointOnEdge(mesh,m_startedge,objpos);
        m_SM->m_workplane.SetPoint(objmat.PointTransform(objpos));
        m_startpos = objpos;
        m_startface = -1;
        m_startsize = cursize;
        m_strokeproc.Start(m_startpos,m_startedge,m_startface,m_startsize);
        m_delaystart = FALSE;
        if (m_type == TYPE_REPLAYEXTEND){
          m_strokeproc.Replay(m_strokeprocTemp, !!m_replayFlip, !!m_replayScale);
          m_lastReplayFlip  = m_replayFlip;
          m_lastReplayScale = m_replayScale;
          m_replayed = TRUE;
        }
      }
    }
    else{
      m_strokeproc.Start(objpos,-1,-1,cursize);
      m_delaystart = FALSE;
    }
  }
  else if (m_type == TYPE_CLONE){
    ProcClone(hit, curposworld, constrscale, vpt);
  }
  else if (m_type == TYPE_INSET){
    ProcInset(hit,objpos,mesh);
  }
  else if (m_type == TYPE_GRABPATH){
    m_drawpointsforce = 2;
  }
  else if (m_type == TYPE_REPLAYEXTEND ||
          m_type == TYPE_REPLAYEXTRUDE)
  {
    if (m_replayed && (m_lastReplayFlip != m_replayFlip || m_lastReplayScale != m_replayScale)){
      m_strokeproc.ResetToStart();
      m_strokeproc.Replay(m_strokeprocTemp, !!m_replayFlip, !!m_replayScale);
      m_lastReplayFlip  = m_replayFlip;
      m_lastReplayScale = m_replayScale;
    }
  }
  else if (m_type == TYPE_REPLAYFREE){
    if (m_replayed){
      m_strokeproc.ResetToStart();
      m_strokeproc.CheckHit(objpos,cursize,vpt);
      m_replayed = FALSE;
    }
    if (!m_replayed && m_strokeproc.HasPoints(3)){
      m_strokeproc.Replay(m_strokeprocTemp, !!m_replayFlip, !!m_replayScale);
      m_lastReplayFlip  = m_replayFlip;
      m_lastReplayScale = m_replayScale;
      m_replayed = TRUE;
    }
    else if (!m_replayed){
      m_strokeproc.CheckHit(objpos,cursize,vpt);
    }
  }
  else if (m_type != TYPE_IGNORE &&
           m_type != TYPE_REPLAYEXTEND &&
           m_type != TYPE_REPLAYEXTRUDE)
  {
    m_strokeproc.CheckHit(objpos,cursize,vpt);

    {
      closestedge = m_type == TYPE_EXTEND && (m_allowConnection && !m_SM->m_ip->SelectionFrozen()) ? GetEndEdge(hit) : -1;
      if (closestedge >= 0){
        Tab <int> verts;
        m_drawhighlights = m_strokeproc.GetEndEdges(closestedge,m_highlightedges,verts,m_highlightanchor);

        int endedgeid = closestedge;
        int lastendpoint;
        SMInterpolation interpolate;
        m_strokeproc.GetStrokePointArray(lastendpoint);

        if (m_lastendedge != endedgeid || m_lastendpoint != lastendpoint || m_lastinterpolate.Equals(m_strokeproc.GetConnectInterpolation(interpolate)))
        {
          m_lastendedge   = endedgeid;
          m_lastendpoint  = lastendpoint;
          m_lastinterpolate = interpolate;
          m_strokeprocTemp = m_strokeproc;
          if (endedgeid >= 0 && m_strokeproc.OverlapsWithStart(endedgeid)){
            m_pmeshTemp.Clear();
          }
          else{
            m_pmeshTemp  = m_pmesh;
            m_strokeprocTemp.ChangeOutput(&m_pmeshTemp);
            m_strokeprocTemp.End(endedgeid, hit.face.fid,
              endedgeid >= 0 ? MNMesh_closestPointOnEdge(m_SM->m_mesh,endedgeid,hit.face.pos) : Point3());
          }
        }
        m_drawconnected = TRUE;
      }
    }
  }
}

SMBrushStrip::Mode SMBrushStrip::GetFaceType(MNMesh* mesh, int face, const Matrix3 &objmatinv,Matrix3 &facematrix)
{

  MNTempData tmp(mesh);
  MNFaceClusters* clusters = tmp.FaceClusters(MN_SEL);
  int cluster = clusters->clust[face];
  Tab<Point3>& norms = *tmp.ClusterNormals(MESH_FACE);
  Point3 normal = norms[cluster].Normalize();
  float angle = cosf(m_modeangle * DEG_TO_RAD);
  facematrix = tmp.ClusterTM(cluster);

  if (abs(DotProd(normal,objmatinv.VectorTransform(m_SM->m_workplane.GetNormal()).Normalize() )) < angle && !m_SM->m_ip->SelectionFrozen())
  {
    return TYPE_EXTRUDE;
  }
  else{
    return TYPE_GRABPATH;
  }
}


BOOL SMBrushStrip::StartInset(const SMHit &hit, MNMesh * mesh )
{
  m_startpos = hit.face.pos;
  m_startmouse = hit.mouse;

  // keep original topology
  m_tchange = new TopoChangeRestore(m_SM->m_polyobj);
  m_tchange->Before();

  m_meshtemp.SetMesh(mesh);
  m_meshtemp.freeAll();

  // extrude cluster
  MNFaceClusters* faceclusters = m_meshtemp.FaceClusters(MN_SEL);
  Tab<int>  &clusters = faceclusters->clust;
  int cluster = clusters[hit.face.fid];

  for (int f = 0; f < mesh->FNum(); f++){
    if (clusters[f] == cluster){
      mesh->F(f)->SetFlag(SM_STRIPFACE_FLAG);
    }
  }

  if (!mesh->ExtrudeFaceCluster(*faceclusters,cluster)){
    return FALSE;
  }

  m_meshtemp.OutlineDir(MESH_EXTRUDE_CLUSTER,SM_STRIPFACE_FLAG);

  // we need vertices for "moving"
  m_vertrestore = new MeshVertRestore(m_SM->m_polyobj);

  m_SM->m_epoly->LocalDataChanged(PART_GEOM|PART_TOPO);

  return TRUE;
}

void SMBrushStrip::ProcInset(const SMHit &hit,  Point3 curpos, MNMesh * __restrict mesh)
{
  if (!m_vertrestore) return;

  Point3 delta = curpos-m_startpos;

  // refill vertices
  m_vertrestore->Restore(FALSE);

  float amount = delta.Length();

  // apply delta
  Tab<Point3> *outDir = m_meshtemp.OutlineDir (MESH_EXTRUDE_CLUSTER, SM_STRIPFACE_FLAG);
  if (outDir && outDir->Count()) {
    Point3* __restrict od = outDir->Addr(0);
    for (int i=0; i<mesh->numv; i++) mesh->v[i].p -= od[i]*amount;
  }


  m_SM->m_epoly->LocalDataChanged(PART_GEOM);
  m_SM->m_polyobj->NotifyDependents(FOREVER,PART_GEOM,REFMSG_CHANGE);
}


void SMBrushStrip::StartClone( const SMHit &hit, MNMesh * mesh )
{
  m_startpos = hit.face.pos;
  m_startmouse = hit.mouse;
  m_center = m_SM->m_ip->GetTransformAxis(m_SM->m_node,0).GetRow(3);

  // keep original topology
  m_tchange = new TopoChangeRestore(m_SM->m_polyobj);
  m_tchange->Before();

  // clone
  // FIXME only current cluster
  mesh->CloneFaces();

  // init for transform
  m_SM->m_polyobj->TransformStart(m_SM->m_ip->GetTime());

  // we need vertices for "moving"
  m_vertrestore = new MeshVertRestore(m_SM->m_polyobj);

  m_SM->m_epoly->LocalDataChanged(PART_GEOM|PART_TOPO);

  // check for flip
  BitArray cursel(mesh->FNum());
  BitArray curelem(mesh->FNum());

  mesh->getFaceSel(cursel);
  mesh->ElementFromFace(hit.face.fid,curelem);

  BitArray combined = cursel & ~curelem;
  m_canflip = combined.IsEmpty();

  m_SM->m_drawtransform = TRUE;
  m_SM->SetTransformPts(m_SM->m_workplane.GetMatrix(),m_startpos);
}

void SMBrushStrip::ProcClone( const SMHit &hit, Point3 curposworld, float constrscale, ViewExp * vpt )
{
  // MODE_CLONE
  Point3 sposworld = hit.objtm.PointTransform(m_startpos);
  Point3 delta = curposworld-sposworld;
  Matrix3 wplaneaxis = m_SM->m_workplane.GetMatrix();
  wplaneaxis.SetRow(3,m_center);

  // refill vertices
  m_vertrestore->Restore(FALSE);

  // scale may need flip
  DWORD flipbits = 0;
  DWORD*  pflipbits = NULL;
  if (m_canflip){
    pflipbits = &flipbits;
  }

  Point3 localchange;
  SMTransform_run(m_SM->m_transformtype,-1,m_SM->m_polyobj,delta,(hit.mouse-m_startmouse)*constrscale,localchange,wplaneaxis,Matrix3(hit.objtm),vpt->GetVPWorldWidth(sposworld),m_SM->m_ip);
  SMTransform_updateStatus(m_SM->m_transformtype,m_SM->m_workplane.GetSwizzled(localchange,!SMTransformType_isScale(m_SM->m_transformtype)),m_SM->m_ip);

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

void SMBrushStrip::StartStroke(int mode,BOOL cont, int subobj, const SMHit &hit)
{
  Matrix3 objmat = hit.objtm;
  MNMesh  *mesh = m_SM->m_mesh;

#ifdef SM_SUPPORT_STRIP_REPLAY
  if (mode == SM_STRIP_REPLAY && m_strokeproc.CanReplay()){
    m_strokeprocTemp = m_strokeproc;
  }
#endif


  // reset preview mesh
  m_lastendedge = -1;
  m_drawconnected = FALSE;
  m_pmesh.Clear();
  m_pmeshTemp.Clear();
  m_strokeproc.PrepStart(mesh,&m_pmesh,&m_SM->m_workplane,objmat);

  // undo/redo
  // create restore object
  SM_ASSERT(!theHold.Holding());
  if (0 && theHold.Holding()) {
    theHold.Cancel();
  }
  m_canflip = FALSE;
  m_isflipped = FALSE;
  m_startface = -1;
  m_startedge = -1;
  m_replayed = FALSE;
  m_refine = FALSE;
  m_type = TYPE_FREE;
  m_subobj = subobj;
  
#ifdef SM_SUPPORT_STRIP_REPLAY
  if (mode == SM_STRIP_REPLAY && m_strokeprocTemp.CanReplay()){
    m_type = TYPE_REPLAYFREE;
  }
#endif
  m_delaystart = TRUE;

  if (SMHit_hasFaceHit(hit))
  {
#ifdef SM_SUPPORT_STRIP_FACEOPS
    if (subobj == SM_SO_FACE && ( MNMesh_isFaceSelected(mesh,hit.face.fid) || 
                                  MNMesh_isFaceSelected(mesh,mesh->E(hit.subobj.eid)->OtherFace(hit.face.fid))))
    { // extrusion or clone
      Matrix3 objmatinv = Inverse(objmat);
      int fid = MNMesh_isFaceSelected(mesh,hit.face.fid) ? hit.face.fid : mesh->E(hit.subobj.eid)->OtherFace(hit.face.fid);
      m_type = GetFaceType(mesh,fid,objmatinv,m_startfacematrix);

      if (m_type == TYPE_EXTRUDE)
      {
        m_SM->m_workplane.SetPoint(objmat.PointTransform(hit.face.pos));
        m_startface = fid;
        m_startedge = hit.subobj.eid;
        m_startnormal = hit.face.normal;
        m_startpos  = hit.face.pos;
        m_startsize  = hit.brush;

#ifdef SM_SUPPORT_STRIP_REPLAY
        if (mode == SM_STRIP_REPLAY && m_strokeprocTemp.CanReplay()){
          m_type = TYPE_REPLAYEXTRUDE;
        }
#endif

        m_delaystart = FALSE; // delay not needed
        m_strokeproc.Start(m_startpos,m_startedge,m_startface,m_startsize,&m_startfacematrix);

        if (m_type == TYPE_REPLAYEXTRUDE){
          m_strokeproc.Replay(m_strokeprocTemp, !!m_replayFlip, !!m_replayScale);
          m_lastReplayFlip  = m_replayFlip;
          m_lastReplayScale = m_replayScale;
          m_replayed = TRUE;
        }

        m_SM->m_epoly->LocalDataChanged(PART_DISPLAY | PART_SELECT);
        m_SM->m_polyobj->NotifyDependents(FOREVER,PART_DISPLAY | PART_SELECT,REFMSG_CHANGE);
      }
      else if (m_type == TYPE_GRABPATH && m_strokeprocTemp.ExtractReplay(hit.subobj.eid,fid,mesh,&m_SM->m_workplane,objmat))
      {
        //m_strokeproc = m_strokeprocTemp;
        m_highlightanchor = m_strokeprocTemp.GetFirstFixedPoint();
      }
#if 0
      else if (m_type == TYPE_CLONE){
        StartClone(hit, mesh);
      }
      else if (m_type == TYPE_INSET){
        StartInset(hit, mesh);
      }
#endif
      else {
        m_type = TYPE_IGNORE;
      }
      m_delaystart = FALSE; // delay not needed
    }
    else
#endif
    if (MNMesh_isFaceOpen(mesh,hit.face.fid))
    { // edge extend
      m_type = TYPE_EXTEND;
#ifdef SM_SUPPORT_STRIP_REPLAY
      if (mode == SM_STRIP_REPLAY && m_strokeprocTemp.CanReplay()){
        m_type = TYPE_REPLAYEXTEND;
      }
#endif

      m_SM->m_workplane.SetPoint(objmat.PointTransform(hit.face.pos));
      m_startface = hit.face.fid;
      m_startnormal = hit.face.normal;
      m_startedge = MNMesh_findClosestEdge(mesh,hit.face.fid,hit.face.pos,-1,true);
      m_startedge = m_startedge < 0 ?  hit.subobj.eid : m_startedge; 
      m_startdist = DotProd(hit.face.normal,hit.face.pos);
    }
  }

  theHold.Begin();

  if (m_type == TYPE_CLONE)
    m_SM->m_polyobj->TransformHoldingStart(m_SM->m_ip->GetTime());


  m_stroking = TRUE;
}

BOOL SMBrushStrip::BeginRefine()
{
  m_refinehit = m_SM->m_hit;
  m_refine = FALSE;
  if (m_SM->m_ip->SelectionFrozen())
    m_allowConnection = FALSE;

  if (m_stroking && !m_delaystart && m_SM->GetMouseTravel() > 7){

    if (m_type == TYPE_FREE || 
        m_type == TYPE_EXTEND || 
        m_type == TYPE_EXTRUDE)
    {
      // process end
      // find openedge

      int endedgeid = (m_type == TYPE_EXTEND && m_allowConnection ) ? GetEndEdge(m_refinehit,false) : -1;

      if (endedgeid >= 0 && m_strokeproc.OverlapsWithStart(endedgeid)){
        return FALSE;
      }

      m_pmeshTemp   = m_pmesh;
      m_pmeshTemp.CollapseDeadStructs();
      m_pmeshTemp.ClearFlag(MN_MESH_FILLED_IN);
      m_pmeshTemp.FillInMesh();

      m_strokeprocTemp = m_strokeproc;

      m_strokeproc.End(endedgeid, m_refinehit.face.fid,
        endedgeid >= 0 ? MNMesh_closestPointOnEdge(m_SM->m_mesh,endedgeid,m_refinehit.face.pos) : Point3());

      m_refine = TRUE;
      m_drawconnected = FALSE;

      return TRUE;
    }
  }
  return FALSE;
}

void SMBrushStrip::Refine()
{
  if (!m_refine) return;

  int endedgeid = (m_type == TYPE_EXTEND && m_allowConnection) ? GetEndEdge(m_refinehit,false) : -1;

  // copy back
  m_pmesh = m_pmeshTemp;
  m_strokeproc.Refine(m_strokeprocTemp);

  m_strokeproc.End(endedgeid, m_refinehit.face.fid,
    endedgeid >= 0 ? MNMesh_closestPointOnEdge(m_SM->m_mesh,endedgeid,m_refinehit.face.pos) : Point3());

  
}

void SMBrushStrip::EndStroke(const SMHit &inhit)
{
  const SMHit &hit = m_refine ? m_refinehit : inhit;

  m_drawpointsforce = FALSE;

  if (m_type == TYPE_IGNORE){
    AbortStroke();
    return;
  }

  if (m_stroking && !m_delaystart && m_type == TYPE_GRABPATH)
  {
    if (theHold.Holding()){
      theHold.Cancel();
    }
    m_strokeproc = m_strokeprocTemp;
    m_replayFlip = FALSE;
  }
  else if (m_stroking && !m_delaystart && m_SM->GetMouseTravel() > 7){
    bool useStrokProc = false;

    if (m_type == TYPE_FREE || 
        m_type == TYPE_EXTEND || 
        m_type == TYPE_EXTRUDE || 
        m_type == TYPE_REPLAYFREE || 
        m_type == TYPE_REPLAYEXTEND ||
        m_type == TYPE_REPLAYEXTRUDE)
    {
      // process end
      // find openedge

      BOOL allowconnection = (m_refine && m_allowConnection) || (!m_refine && m_allowConnection && !m_SM->m_ip->SelectionFrozen());

      int endedgeid = (m_type == TYPE_EXTEND && allowconnection) ? GetEndEdge(hit,false) : -1;

      if (endedgeid >= 0 && m_strokeproc.OverlapsWithStart(endedgeid)){
        // treat as abort
        AbortStroke();
        return;
      }

      if (m_type == TYPE_FREE ||
          m_type == TYPE_EXTEND || 
          m_type == TYPE_EXTRUDE)
      {
        if(m_refine){
          // copy back
          m_pmesh = m_pmeshTemp;
          m_strokeproc.Refine(m_strokeprocTemp);
        }

        m_strokeproc.End(endedgeid, hit.face.fid,
          endedgeid >= 0 ? MNMesh_closestPointOnEdge(m_SM->m_mesh,endedgeid,hit.face.pos) : Point3());
      }

      useStrokProc = true;
    }

    if (theHold.Holding()){
      //theHold.Put (new ComponentFlagRestore (m_SM->m_polyobj, SMSOTypeToMNType(m_subobj)));

      if (useStrokProc && m_pmesh.FNum())
      {
        // attach and merge final mesh
        m_tchange = new TopoChangeRestore(m_SM->m_polyobj);
        m_tchange->Before();

        m_strokeproc.ApplyChanges(SM_SO_FACE, m_selection);
        m_strokeproc.Clear();
      }
      if (m_tchange){
        m_tchange->After();
        theHold.Put (m_tchange);
      }
      m_tchange = NULL;

      if (m_type == TYPE_CLONE){
        m_SM->m_polyobj->TransformHoldingFinish(m_SM->m_ip->GetTime());

        delete m_vertrestore;
        m_vertrestore = NULL;

        m_SM->m_drawtransform = FALSE;
      }
      else if (m_type == TYPE_INSET){

        delete m_vertrestore;
        m_vertrestore = NULL;

        m_meshtemp.freeAll();
      }
      else if ( m_type == TYPE_REPLAYEXTEND ||
                m_type == TYPE_REPLAYFREE ||
                m_type == TYPE_REPLAYEXTRUDE)
      {
        m_strokeproc = m_strokeprocTemp;
      }
      else if (useStrokProc){
#ifdef SM_SUPPORT_STRIP_REPLAY
        if (m_strokeproc.CanReplay()){
          m_strokeprocTemp = m_strokeproc;
          m_replayFlip = FALSE;
        }
#endif
      }
    } // this is a comment
    m_SM->m_epoly->CollapseDeadStructs();
    m_SM->m_mesh->PrepForPipeline();
    theHold.Accept(_M("AIF Strip"));

    if (m_type == TYPE_CLONE){
      m_SM->m_polyobj->TransformFinish(m_SM->m_ip->GetTime());
    }
  }
  else{
    AbortStroke();
    return;
  }

  m_refine = FALSE;
  m_type = TYPE_FREE;
  m_stroking = FALSE;
  m_pmesh.Clear();
  m_pmeshTemp.Clear();

  m_SM->m_epoly->LocalDataChanged(PART_GEOM | PART_TOPO | PART_TEXMAP | PART_VERTCOLOR | PART_SELECT | PART_DISPLAY);
  m_SM->m_fullredraw = TRUE;
}
void SMBrushStrip::AbortStroke()
{
  if (theHold.Holding()){
    if (m_type == TYPE_CLONE || m_type == TYPE_INSET){
      SM_ASSERT(m_tchange);

      m_tchange->After();
      theHold.Put (m_tchange);
      m_tchange = NULL;

      if (m_vertrestore){
        delete m_vertrestore;
        m_vertrestore = NULL;
      }
    }
    else if ((m_type == TYPE_FREE || 
              m_type == TYPE_EXTEND || 
              m_type == TYPE_EXTRUDE || 
              m_type == TYPE_REPLAYFREE || 
              m_type == TYPE_REPLAYEXTEND ||
              m_type == TYPE_REPLAYEXTRUDE) &&
       m_pmesh.FNum())
    {
      m_strokeproc.DiscardChanges();
    }

    m_strokeproc = m_strokeprocTemp;
  }


  m_pmesh.Clear();
  m_pmeshTemp.Clear();

  if (theHold.Holding()) {
    theHold.Cancel();
  }

  if (m_type == TYPE_CLONE){
    m_SM->m_polyobj->TransformCancel(m_SM->m_ip->GetTime());
    m_SM->m_drawtransform = FALSE;
  }
  else if (m_type == TYPE_INSET){
    m_meshtemp.freeAll();
  }

  m_refine = FALSE;
  m_type = TYPE_FREE;
  m_stroking = FALSE;
  m_drawpointsforce = FALSE;

  m_SM->m_epoly->LocalDataChanged(PART_GEOM | PART_TOPO | PART_TEXMAP | PART_VERTCOLOR | PART_SELECT | PART_DISPLAY);
  m_SM->m_fullredraw = TRUE;
}



#endif

