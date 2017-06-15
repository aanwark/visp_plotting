#ifndef VPCUSTOMFEATURE_H
#define VPCUSTOMFEATURE_H

#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>


#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

class vpCustomFeature : public vpBasicFeature
{
 private:
  double A_star;

 public:
  vpCustomFeature();
  // Destructor
  virtual ~vpCustomFeature() {}

  //dummy1 and dummy2 are zero values
  void buildFrom (const double angle, const double u, const double v,
                  const double Area, const double dummy1, const double dummy2);
  void buildFrom (const double angle, const double u, const double v,
                  const double Area, const double dummy1, const double dummy2,
                  const double A_star);

  vpColVector error (const vpBasicFeature &s_star);
                     //const unsigned int select = FEATURE_ALL);

  double getAngle() const { return s[0] ; }

  double getU() const { return s[1]; }

  double getV() const { return s[2]; }

  double getArea() const { return s[3]; }

  void init();

  vpMatrix interaction (const unsigned int select = FEATURE_ALL);

  void print () const;

  void setValues (const double angle, const double u, const double v,
                  const double Area, const double dummy1, const double dummy2);

  void setA_star (const double A_star);

};

#endif
