#include <visp3/visual_features/vpBasicFeature.h>
#include "vpCustomFeature.h"

// Exception
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

// simple math function (round)
#include <visp3/core/vpMath.h>

// Display Issue

// Meter/pixel conversion
#include <visp3/core/vpCameraParameters.h>

//Color / image / display
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <visp3/core/vpFeatureDisplay.h>


void
vpCustomFeature::init()
{
  dim_s = 1 ;
  nbParameters = 6;

  s.resize(dim_s) ;

  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++) flags[i] = false;

    A_star =  0.0 ;
}

vpCustomFeature::vpCustomFeature() : A_star(0)
{
  init();
}


void vpCustomFeature::setValues (const double angle, const double u, const double v, const double Area, const double dummy1, const double dummy2)
{
  s[0] = angle;
  s[1] = u;
  s[2] = v;
  s[3] = Area;
  s[4] = dummy1;
  s[5] = dummy2;

  for (int i = 0; i < 6; i++) flags[i] = true;
}

void vpCustomFeature::setA_star (const double A_star_)
{
  this->A_star = A_star_;
  for (unsigned int i = 6; i < nbParameters; i++) flags[i] = true;
}

// Interaction Matrix

vpColVector
vpCustomFeature::error (const vpBasicFeature &s_star)
{
  vpColVector e(0), eangle(1), eu(1), ev(1), eArea(1), edummy1(1), edummy2(1);
  eangle[0] = s[0] - s_star[0];
  e = vpColVector::stack(e,eangle) ;

  eu[0] = s[1] - s_star[1];
  e = vpColVector::stack(e,eu);

  ev[0] = s[2] - s_star[2];
  e = vpColVector::stack(e,ev);

  eArea[0] = s[3] - s_star[3];
  e = vpColVector::stack(e,eArea);

  edummy1[0] = s[4] - s_star[4];
  e = vpColVector::stack(e,edummy1);

  edummy2[0] = s[5] - s_star[5];
  e = vpColVector::stack(e,edummy2);

  return e;
}

void
vpCustomFeature::print () const
{
  std::cout << "\tangle: " << s[0] << std::endl;
  std::cout << "\tu: " << s[1] << std::endl;
  std::cout << "\tv: " << s[2] << std::endl;
  std::cout << "\tArea: " << s[3] << std::endl;
  std::cout << "\tdummy1: " << s[4] << std::endl;
  std::cout << "\tdummy2: " << s[5] << std::endl;
}

void
vpCustomFeature::buildFrom (const double angle, const double u, const double v,
                  const double Area, const double dummy1, const double dummy2)
{
  s[0] = angle;
  s[1] = u;
  s[2] = v;
  s[3] = Area;
  s[4] = dummy1;
  s[5] = dummy2;
  for (int i = 0; i < 6; i++) flags[i] = true;
}

void
vpCustomFeature::buildFrom (const double angle, const double u, const double v,
                  const double Area, const double dummy1, const double dummy2,
                  const double A_star_)
{
  s[0] = angle;
  s[1] = u;
  s[2] = v;
  s[3] = Area;
  s[4] = dummy1;
  s[5] = dummy2;
  this->A_star = A_star_;
  for (int i = 0; i < 6; i++) flags[i] = true;
}
