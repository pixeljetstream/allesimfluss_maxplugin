/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/
#include "workplane.h"
#include "helpers.h"
#include "collision.h"

void WorkPlane::EnableConstrained(Point3 &refpt, int maxaxis)
{
  m_constrainref = refpt;
  m_constrain = TRUE;
  m_mconstrvalid = FALSE;
  m_constrmaxaxis = maxaxis;
  m_constrscalecur = 1.0f;
  m_constrainmat = m_mat;
  m_constrainmat.SetRow(3,refpt);
}
void WorkPlane::DisableConstrained()
{
  m_constrain = FALSE;
  m_constrscalecur = 1.0f;
}

BOOL WorkPlane::RunConstrain(BOOL validhit,BOOL doit, Point3 &pt)
{
  if (! (doit && validhit)){
    m_constrscalecur = 1.0f;
    return validhit;
  }

  // project pt on refpt plane
  Point3 refnormal = m_mat.GetRow(2);
  float refdot = DotProd(m_constrainref,refnormal);
  Point3 proj;
  float travel = ProjectPlanePoint(refnormal,refdot,pt,proj);

  // get angles for x and y axis
  Point3 dir = (pt-m_constrainref);
  float dirlen = dir.LengthUnify();

  int dirused = m_constrmaxaxis;
  Point3  dirs[4] = {
    m_mat.GetRow(0),
    m_mat.GetRow(1),
    ((m_mat.GetRow(0)+m_mat.GetRow(1))*0.5f).Normalize(),
    ((m_mat.GetRow(0)-m_mat.GetRow(1))*0.5f).Normalize(),
  };

  if (m_mconstrvalid && m_mdeltastart.Length() < m_constrcutoff * 0.5f){
    m_mconstrvalid = FALSE;
  }

  Point3 newdir;
  if (m_mconstrvalid ){// && m_mdeltastart.Length() < m_constrcutoff){
    newdir = m_constrlastdir;
  }
  else{
    // find closest angle
    newdir = Point3_getDirAll(dirs,dirused,dir);

    if (m_mdeltastart.Length() >= m_constrcutoff){
      m_constrlastdir = newdir;
      m_mconstrvalid = TRUE;
    }
  }
  m_constrscalecur = DotProd(newdir,dir);
  dirlen *= m_constrscalecur;

  // use angle and offset to original height
  pt = m_constrainref + (newdir*dirlen) + (travel*refnormal);

  return true;
}

void WorkPlane::SetPoint(Point3 &pt)
{
  m_mat.SetRow(3,pt);
  m_plane = Plane(m_mat.GetRow(2),pt);
  m_matinv = Inverse(m_mat);
}


BOOL WorkPlane::Intersect(const Ray &ray, Point3 &out, BOOL allowconstrain)
{
  BOOL validhit = PlaneIntersect(ray,m_plane.GetNormal(),-m_plane.GetPlaneConstant(),out);
  validhit = RunConstrain(validhit,allowconstrain && m_constrain,out);
  return validhit;
}
BOOL WorkPlane::Intersect(const Ray &ray, Point3 &out, const Point3 &planepos, BOOL allowconstrain)
{
  BOOL validhit = PlaneIntersect(ray,m_plane.GetNormal(),planepos,out);
  validhit = RunConstrain(validhit,allowconstrain && m_constrain,out);
  return validhit;
}
BOOL WorkPlane::Intersect(const Ray &ray, Point3 &out, float planedot, BOOL allowconstrain)
{
  BOOL validhit = PlaneIntersect(ray,m_mat.GetRow(2),planedot,out);
  validhit = RunConstrain(validhit,allowconstrain && m_constrain,out);
  return validhit;
}
void WorkPlane::Update(Interface *ip, ViewExp *vpt, INode *active, MNMesh *mesh, const SMHit &mhit)
{
  Matrix3 viewmat;
  vpt->GetAffineTM(viewmat);
  viewmat = Inverse(viewmat);

  m_islocal = FALSE;

  // find out which axis fits best
  Matrix3 coordsys;

  int refsys = ip->GetRefCoordSys();
  switch(refsys)
  {
  default:
  case COORDS_HYBRID:
    coordsys = Matrix3(TRUE);
    // world aligned but current node as center
    if (active){

      Matrix3 objtm = ip->GetTransformAxis(active,0); //active->GetObjectTM(ip->GetTime());
      //coordsys.SetRow(3,objtm.GetRow(3));
      coordsys = objtm;

    }
    break;
  case COORDS_SCREEN:
    coordsys = viewmat;
    if (active){
      Matrix3 objtm = active->GetObjectTM(ip->GetTime());
      Point3  camtoobj = objtm.GetRow(3)-viewmat.GetRow(3);

      coordsys.SetRow(3,viewmat.GetRow(3)+(viewmat.GetRow(2)*DotProd(viewmat.GetRow(2),camtoobj)));
    }
    else{
      // point on workplane
      GraphicsWindow  *gw = vpt->getGW();
      IPoint2 dim(gw->getWinSizeX(),gw->getWinSizeY());
      coordsys.SetRow(3,vpt->MapCPToWorld(vpt->GetPointOnCP(dim/2)));
    }
    break;
  case COORDS_GIMBAL:
  case COORDS_WORLD:
    coordsys = Matrix3(TRUE);
    break;
  case COORDS_PARENT:
    if (!active){
      coordsys = Matrix3(TRUE);
      break;
    }

    if (ip->GetSubObjectLevel() > 0){
      coordsys = active->GetObjectTM(ip->GetTime());
    }
    else if (active->GetParentNode()){
      coordsys = active->GetParentNode()->GetObjectTM(ip->GetTime());
    }
    else{
      coordsys = Matrix3(TRUE);
    }
    break;
  case COORDS_LOCAL:
    if (!active){
      coordsys = Matrix3(TRUE);
      break;
    }

    if (mesh && ip->GetSubObjectLevel() > 0 && SMHit_hasFaceHit(mhit)){
      Matrix3 objtm = active->GetObjectTM(ip->GetTime());
      if (ip->GetSubObjectLevel() == SM_SO_FACE && MNMesh_isFaceSelected(mesh,mhit.subobj.fid)){
        MNTempData tmp(mesh);
        MNFaceClusters* clusters = tmp.FaceClusters(MN_SEL);
        int cluster = clusters->clust[mhit.subobj.fid];
        coordsys = ip->GetTransformAxis(active,cluster);
        coordsys.SetRow(3, objtm.PointTransform(mhit.face.pos) );
      }
      else{
        coordsys = MNMesh_getFaceMatrix(mesh,mhit.subobj.fid,mhit.subobj.eid,mhit.face.pos);
        coordsys = coordsys * objtm;
      }

      m_islocal = TRUE;
    }
    else{
      coordsys = active->GetObjectTM(ip->GetTime());
    }
    break;
  case COORDS_OBJECT:
  {
    Interface7 *ip7 = GetCOREInterface7();

    if (ip7->GetRefCoordNode()){
      coordsys = ip7->GetRefCoordNode()->GetObjectTM(ip->GetTime());
    }
    else if (ip->GetActiveGrid()){
      coordsys = ip->GetActiveGrid()->GetObjectTM(ip->GetTime());
    }
    else{
      coordsys = Matrix3(TRUE);
    }
  }
    break;
#if MAX_RELEASE >= 10000
  case COORDS_WORKINGPIVOT:
  {
    IWorkingPivot* wp = GetIWorkingPivot();
    if (wp){
      coordsys = wp->GetTM();
    }
    else{
      coordsys = Matrix3(1);
    }
  }
    break;
#endif
  }

  bool coordchanged = false;
  if (DotProd(coordsys.GetRow(0),m_lastcoordsys.GetRow(0)) < 0.9999f || 
      DotProd(coordsys.GetRow(1),m_lastcoordsys.GetRow(1)) < 0.9999f ||
      DotProd(coordsys.GetRow(2),m_lastcoordsys.GetRow(2)) < 0.9999f)
  {
    m_lastcoordsys = coordsys;
    coordchanged = true;
  }

  coordsys.Orthogonalize();
  coordsys.NoScale();

  m_coordsys = coordsys;
  m_viewmat = viewmat;
  m_viewdir = viewmat.GetRow(2);
  m_viewpos = viewmat.GetRow(3);
  m_viewortho = !vpt->IsPerspView();

  bool viewchanged = false;
  if (DotProd(m_viewdir,m_lastviewdir) < 0.9999f){
    m_lastviewdir = m_viewdir;
    viewchanged = true;
  }
  
  Matrix3 refmat = m_lastrefmat;
  if (viewchanged) {
    // reset everything
    m_lastconstrain = ip->GetAxisConstraints();
    refmat = m_viewmat;
  }
  else if (coordchanged){
    // keep close to last world normal
    m_lastconstrain = ip->GetAxisConstraints();
    Matrix3 temp(1);
    Point3  oldnormal = GetNormal();
    float angle = -2.0f;
    int   alt   = 0;
    for (int i = 0; i < 3; i++){
      MakeMatrix(refmat,temp,m_viewmat,i,NULL,NULL);
      float curangle = abs(DotProd(oldnormal,refmat.GetRow(2)));
      if ( curangle > angle){
        alt = i;
        angle = curangle;
      }
    }
    MakeMatrix(refmat,temp,m_viewmat,alt,NULL,NULL);
  }

#if 0
  smul = 1.0f;
  currow = (Matrix3_getDirRow(m_coordsys,refdir,smul) + 1) * ((smul > 0) ? 1 : -1) ;
  if ( currow != m_lastrow){
    m_lastrow = currow;
    m_lastconstrain = ip->GetAxisConstraints();
  }
#endif
  m_useAlternative = (ip->GetAxisConstraints() - m_lastconstrain + 6) % 3;

  UpdateInternals(refmat);
}
void WorkPlane::MakeMatrix(Matrix3 &mat, const Matrix3 &coordsys, const Matrix3 &refmat, int alternative, int *outswizzle, float *outsigns)
{
  // The Z axis of this matrix is the view direction.
  float sx,sy,sz;
  BitArray visited(3);

  BitArray old = visited;
  int rowz = Matrix3_getDirRow(coordsys,refmat.GetRow(2),sz,visited);
  for (int i = 0; i < alternative; i++){
    old = visited;
    rowz = Matrix3_getDirRow(coordsys,refmat.GetRow(2),sz,visited);
  }
  visited &= ~old;

  int rowx = Matrix3_getDirRow(coordsys,refmat.GetRow(0),sx,visited);
  int rowy = Matrix3_getDirRow(coordsys,refmat.GetRow(1),sy,visited);

  if (outswizzle){
    outswizzle[0] = rowx;
    outswizzle[1] = rowy;
    outswizzle[2] = rowz;
  }

  // align to view
  if (DotProd(coordsys.GetRow(rowz)*sz,m_viewdir) < 0.0f) sz *= -1.0f;
  if (DotProd(coordsys.GetRow(rowy)*sy,m_viewmat.GetRow(1)) < 0.0f) sy *= -1.0f;

  Point3 z = coordsys.GetRow(rowz)*sz;
  Point3 y = coordsys.GetRow(rowy)*sy;

  mat.SetRow(3,coordsys.GetRow(3));
  mat.SetRow(2,z);
  mat.SetRow(1,y);
  mat.SetRow(0,CrossProd(y,z));

  if (outsigns){
    outsigns[0] = DotProd(mat.GetRow(0),coordsys.GetRow(rowx)) > 0.0f ? 1.0f : -1.0f;
    outsigns[1] = sy;
    outsigns[2] = sz;
  }
}

void WorkPlane::UpdateInternals(const Matrix3 &refmat)
{
  MakeMatrix(m_mat,m_coordsys,refmat,m_useAlternative,m_outswizzle,m_outsigns);
  m_lastrefmat = refmat;

  m_plane = Plane(m_mat.GetRow(2),m_mat.GetRow(3));
  m_matinv = Inverse(m_mat);

  m_mconstrvalid = FALSE;
  m_constrainmat = m_mat;
  m_constrainmat.SetRow(3,m_constrainref);
}

Point3 WorkPlane::GetSwizzled(const Point3 &p, bool signs)
{
  Point3 pt;

  pt[m_outswizzle[0]] = p.x;
  pt[m_outswizzle[1]] = p.y;
  pt[m_outswizzle[2]] = p.z;
  //pt[0] = p[m_outswizzle[0]];
  //pt[1] = p[m_outswizzle[1]];
  //pt[2] = p[m_outswizzle[2]];

  if (signs){
    pt[m_outswizzle[0]] *= m_outsigns[0];
    pt[m_outswizzle[1]] *= m_outsigns[1];
    pt[m_outswizzle[2]] *= m_outsigns[2];
  }

  return pt;
}

void WorkPlane::UpdateMouse(const Point2 &curmouse, const Point2 &deltastart, GraphicsWindow *gw)
{
  if (m_constrain){
    Point3 out;
    gw->setTransform(Matrix3(1));
    gw->transPoint(&m_constrainref,&out);
    m_mdeltastart = curmouse-((Point2)out);
  }
  else{
    m_mdeltastart = deltastart;
  }
  m_mpos = curmouse;

}

Ray   WorkPlane::GetRayFromPoint(const Point3 &inpt)
{
  Ray ray;

  if (m_viewortho){
    ProjectPlanePoint(m_viewdir,DotProd(m_viewpos,m_viewdir),inpt,ray.p);
    ray.dir = m_viewdir;
  }
  else{
    ray.p = m_viewpos;
    ray.dir = (inpt-ray.p).Normalize();
  }

  return ray;
}

Point3 WorkPlane::CorrectPointVisible( const Point3 &inpt, bool applyoffset)
{
  if (!m_collsys){
    return inpt;
  }

  Point3 outpt = inpt;
  Ray ray = GetRayFromPoint(inpt);

  m_collsys->Collide(ray,outpt);
  return outpt;
}

BOOL WorkPlane::CorrectNormalVisible( const Point3 &inpt, Point3 &normal )
{
  if (!m_collsys){
    return FALSE;
  }

  Point3 outpt = inpt;
  Ray ray = GetRayFromPoint(inpt);


  return m_collsys->Collide(ray,outpt,normal);
}

BOOL WorkPlane::CorrectIntersect( BOOL validhit,const Ray &oldray, Point3 &outpt, BOOL &intersect, BOOL applyoffset )
{
  if (!m_collsys) return validhit;
  BOOL collhit = FALSE;

  Ray ray = validhit ? GetRayFromPoint(outpt) : oldray;

  if (applyoffset && m_colloffset != 0.0f){
    Point3 normal;
    if ((collhit=m_collsys->Collide(ray,outpt,normal))==TRUE){
      outpt += normal*m_colloffset;
    }
  }
  else{
    collhit=m_collsys->Collide(ray,outpt);
  }
  intersect = collhit;

  return validhit || collhit;
}

__forceinline BOOL WorkPlane::CorrectProjection( Point3 &out, const Point3 &normal )
{
  Ray ray;
  Point3 collnormal;
  ray.p = out + (normal*m_collprojoffset);
  ray.dir = -normal;

  if (m_collsys->Collide(ray,out,collnormal)){
    out += collnormal*m_colloffset;
    return TRUE;
  }
  return FALSE;
}

BOOL WorkPlane::CorrectProjection( Point3 &out, Point3 normal, const Matrix3& mat, const Matrix3& matinv )
{
  out = mat.PointTransform(out);
  normal = mat.VectorTransform(normal);
  normal = normal.Normalize();
  BOOL res = CorrectProjection(out,normal);
  out = matinv.PointTransform(out);

  return res;
}

void WorkPlane::ResetOnUpdate()
{
  m_viewdir = Point3(0,0,0);
}
