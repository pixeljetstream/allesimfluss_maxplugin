/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_WORKPLANE__H
#define __SKETCHMODELER_WORKPLANE__H

#include "maxincludes.h"
#include "helpers.h"

#if MAX_RELEASE >= 10000
  #include <IWorkingPivot.h>
#endif

class WorkConstraint
{
  BOOL  m_delayedlock;

  int   m_lockaxis;
  BOOL  m_halfaxis;
};

class CollisionSys;

class WorkPlane
{
private:
  void UpdateInternals(const Matrix3 &refmat);

  // changed by updateinternals
  Matrix3 m_mat;    // worldcoords
  Matrix3 m_matinv;
  Plane m_plane;
  int   m_outswizzle[3];
  float m_outsigns[3];

  // changed by update
  Matrix3 m_coordsys;
  Matrix3 m_viewmat;
  Point3  m_viewdir;
  Point3  m_viewpos;
  BOOL  m_viewortho;
  BOOL  m_islocal;

  // changed by update mouse
  Point2  m_mpos;
  Point2  m_mdeltastart;

  // other
  Point3  m_constrainref;
  Matrix3 m_constrainmat;
  BOOL  m_constrain;
  BOOL  m_mconstrvalid;
  Point2  m_mconstr;
  Point3  m_constrlastdir;
  float m_constrscalecur;
  int   m_constrmaxaxis;

  CollisionSys* m_collsys;

  BOOL  m_collalign;
  float m_colloffset;
  float m_collprojoffset;

  Point3  m_lastviewdir;
  Matrix3 m_lastrefmat;
  Matrix3 m_lastcoordsys;
  int     m_lastconstrain;
  int     m_lastrow;

  int  m_useAlternative;

public:
  WorkPlane() : m_constrcutoff(40.0f),m_constrscalecur(1.0f),m_colloffset(0.0f),m_collsys(NULL),m_collalign(TRUE),m_collprojoffset(0.001f),m_useAlternative(0),m_lastrow(0) {}
  ~WorkPlane() {}

  float   m_constrcutoff;

  void ResetOnUpdate();

  void EnableConstrained(Point3 &refpt,int maxaxis=4);
  void DisableConstrained();

  inline void CollEnable(CollisionSys* collcache){ m_collsys = collcache;}
  inline void CollDisable() {m_collsys = NULL;}
  inline void CollSetOffset(float val){ m_colloffset = val;}
  inline float CollGetOffset() const { return m_colloffset;}
  inline void CollSetProjOffset(float val){ m_collprojoffset = val;}
  inline float CollGetProjOffset() const { return m_collprojoffset;}
  inline void CollSetAlign(BOOL val) {m_collalign = val;}
  inline BOOL CollGetAlign() const { return m_collalign;}

  BOOL RunConstrain(BOOL validhit, BOOL doit, Point3 &pt);

  BOOL Intersect(const Ray &ray, Point3 &out, BOOL allowconstrain=FALSE);
  BOOL Intersect(const Ray &ray, Point3 &out, const Point3 &planepos, BOOL allowconstrain=FALSE);
  BOOL Intersect(const Ray &ray, Point3 &out, float planedot, BOOL allowconstrain=FALSE);


  void Update(Interface *ip, ViewExp *vpt, INode *active, MNMesh *mesh, const SMHit &mhit);
  void UpdateMouse(const Point2 &curmouse, const Point2 &deltastart, GraphicsWindow *gw);


  inline BOOL IsLocal() const {return m_islocal;}
  inline BOOL IsConstrained() const { return m_constrain; }

  inline Point3 GetNormal() {return m_plane.GetNormal();}
  inline const Point3& GetConstrainRef() const { return m_constrainref; }
  inline int GetConstrainMaxAxis() const { return m_constrmaxaxis; }
  inline const Matrix3& GetMatrix() const {return m_mat;}
  inline const Matrix3& GetMatrixInv() const {return m_matinv;}
  inline void SetConstrainMatrix(const Matrix3& mat){m_constrainmat = mat;}
  inline const Matrix3& GetConstrainMatrix() const {return m_constrainmat;}

  inline float GetLastConstrainScale() const { return m_constrscalecur; }

  inline BOOL HasCollCorrection() const { return m_collsys != NULL;}
  inline BOOL HasCollAlignment() const { return m_collsys != NULL && m_collalign;}
  Point3 CorrectPointVisible(const Point3 &inpt, bool applyoffset=false);
  BOOL CorrectNormalVisible(const Point3 &inpt, Point3 &normal);
  BOOL CorrectIntersect(BOOL validhit,const Ray &ray, Point3 &out, BOOL &intersect, BOOL offset=false);
  BOOL CorrectProjection(Point3 &out, const Point3 &normal);
  BOOL CorrectProjection(Point3 &out, Point3 normal, const Matrix3& mat, const Matrix3& matinv);

  void MakeMatrix(Matrix3 &mat,const Matrix3 &coordsys, const Matrix3 &refmat, int alternative, int *outswizzle=NULL, float *outsigns=NULL);

  void SetPoint(Point3 &pt);
  inline const Point3& GetPoint() {return m_plane.GetPoint();}

  Point3 GetSwizzled(const Point3 &pt, bool signs=false);

  const Point3& GetViewDir() { return m_viewdir;}
  BOOL GetViewParams(Point3 &viewdir, Point3 &viewpos) { 
    viewdir = m_viewdir;
    viewpos = m_viewpos;
    return m_viewortho;
  }
  Ray GetRayFromPoint(const Point3 &inpt);

};

#endif


