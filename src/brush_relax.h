/*
Alles Im Fluss - Modelling Toolkit
Copyright (C) 2008-2014 Christoph Kubisch. See Copyright Notice in LICENSE file
*/

#ifndef __SKETCHMODELER_BRUSH_RELAX__H
#define __SKETCHMODELER_BRUSH_RELAX__H

#include "brush.h"

#ifdef SM_SUPPORT_RELAX

class SMBrushRelax : public SMBrush
{

public:
  SMBrushRelax() : 
      SMBrush(SM_BRUSH_RELAX, SM_REQUIRE_DRAWHITS | SM_REQUIRE_NOCUSTOMRCS | SM_REQUIRE_DRAWHITSALT) {}
      ~SMBrushRelax(){}

};

#else

class SMBrushRelax : public SMBrush
{

public:
  SMBrushRelax() : 
      SMBrush(SM_BRUSH_RELAX, SM_REQUIRE_DRAWHITS | SM_REQUIRE_NOCUSTOMRCS | SM_REQUIRE_DRAWHITSALT) {}
      ~SMBrushRelax(){}

};

#endif

#endif

