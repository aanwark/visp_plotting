#ifndef VPCUSTOMFEATURE_H
#define VPCUSTOMFEATURE_H

#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>


#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

class vpCustomFeature  : public vpBasicFeature
{
 private:
  double A_star;

 public:
  vpCustomFeature();
  // Destructor
  virtual ~vpCustomFeature() {};

  // void init() ;

  // void display(const vpCameraParameters &cam,
  //              const vpImage<unsigned char> &I,
  //              const vpColor &color=vpColor::green,
  //              unsigned int thickness=1) const {};
  // void display(const vpCameraParameters &cam,
  //              const vpImage<vpRGBa> &I,
  //              const vpColor &color=vpColor::green,
  //              unsigned int thickness=1) const {} ;

  // void print (const unsigned int select= FEATURE_ALL) const {};
  // vpMatrix interaction (const unsigned int select= FEATURE_ALL) {vpMatrix a; return a;};
  // vpCustomFeature *duplicate() const {return 0;};


  //dummy1 and dummy2 are zero values
  void buildFrom (const double angle, const double u, const double v,
                  const double Area, const double dummy1, const double dummy2);
  void buildFrom (const double angle, const double u, const double v,
                  const double Area, const double dummy1, const double dummy2,
                  const double A_star);
  void display(const vpCameraParameters &cam,
               const vpImage<unsigned char> &I,
               const vpColor &color=vpColor::green,
               unsigned int thickness=1) const ;
  void display(const vpCameraParameters &cam,
               const vpImage<vpRGBa> &I,
               const vpColor &color=vpColor::green,
               unsigned int thickness=1) const ;

  vpCustomFeature *duplicate() const ;

  vpColVector error (const vpBasicFeature &s_star,
                     const unsigned int select = FEATURE_ALL);

  double getAngle() const { return s[0] ; }

  double getU() const { return s[1]; }

  double getV() const { return s[2]; }

  double getArea() const { return s[3]; }

  double getdummy1() const { return s[4]; }

  double getdummy2() const { return s[5]; }

  void init();

  vpMatrix interaction (const unsigned int select= FEATURE_ALL);

  void print (const unsigned int select= FEATURE_ALL) const;

  void setValues (const double angle, const double u, const double v,
                  const double Area, const double dummy1, const double dummy2);

  void setA_star (const double A_star);

 public:
  static unsigned int selectAll() {return FEATURE_ALL;}

};

#endif
