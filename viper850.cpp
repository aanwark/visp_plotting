/*! \example tutorial-ibvs-4pts-wireframe-camera.cpp */
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/robot/vpWireFrameSimulator.h>
#include <visp3/robot/vpSimulatorViper850.h>
#include "vpCustomFeature.hpp"
#include <vector>
#include <cmath>

#define PI 3.14159265

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point,
                        const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam);

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point,
                        const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam)
{
  static std::vector<vpImagePoint> traj[4];
  vpImagePoint cog;
  for (unsigned int i=0; i<4; i++) {
    // Project the point at the given camera position
    point[i].project(cMo);
    vpMeterPixelConversion::convertPoint(cam, point[i].get_x(), point[i].get_y(), cog);
    traj[i].push_back(cog);
  }
  for (unsigned int i=0; i<4; i++) {
    for (unsigned int j=1; j<traj[i].size(); j++) {
      vpDisplay::displayLine(I, traj[i][j-1], traj[i][j], vpColor::green);
    }
  }
}

std::vector<double> get_features (vpFeaturePoint p[4])
{
  //vpCustomFeature p_hat;
  double angle, u, v, Area;
  std::vector<double> final;

  //get angle
  angle = atan2 (p[3].get_y() - p[0].get_y(), p[3].get_x() - p[0].get_x());

  if (abs(angle) > PI / 2)
    angle -= PI / 2;

  // std::cout << "Angle: " << angle << std::endl;

  //get center
  u = (p[0].get_x() + p[1].get_x()) / 2;
  v = (p[0].get_y() + p[3].get_y()) / 2;

  double vec1X = p[0].get_x() - p[1].get_x();
  double vec2Y = p[2].get_y() - p[1].get_y();
  double vec2X = p[2].get_x() - p[1].get_x();
  double vec1Y = p[0].get_y() - p[1].get_y();

  Area = fabs(vec1X * vec2Y -  vec2X * vec1Y) * 1;
  //std::cout << "Area: " << Area << std::endl;

  final.push_back(u); final.push_back (v);
  final.push_back(Area); final.push_back (0);
  final.push_back(0); final.push_back (angle);

  return final;
}

int main()
{
  #if defined(VISP_HAVE_PTHREAD)
  try {
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 0.9, vpMath::rad(0), vpMath::rad(0), vpMath::rad(-30));
    vpHomogeneousMatrix wMo(vpTranslationVector(0.40, 0, -0.15),
                            vpRotationMatrix(vpRxyzVector(-M_PI, 0, M_PI/2.)));


    std::vector<vpPoint> point(4) ;
    point[0].setWorldCoordinates(-0.1,-0.1, 0);
    point[1].setWorldCoordinates( 0.1,-0.1, 0);
    point[2].setWorldCoordinates( 0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    vpServo task ;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(-2.0); //0.5

    vpFeaturePoint p[4], pd[4] ;
    for (unsigned int i = 0 ; i < 4 ; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      // pd[i].print();
      // task.addFeature(p[i], pd[i]);
     }

    vpCustomFeature f, fd;
    std::vector<double> v1, v2;
    v1 = get_features (p);
    v2 = get_features (pd);
    f.buildFrom (v1.at(0), v1.at(1), v1.at(2), v1.at(3), v1.at(4), v1.at(5));
    fd.buildFrom (v2.at(0), v2.at(1), v2.at(2), v2.at(3), v2.at(4), v2.at(5));


    std::cout << "Current: " << std::endl;
    for (unsigned int i = 0; i < 6; i++){
      std::cout  << v1.at(i) << "\t";
      if (i == 5) std::cout << std::endl;
    }

    std::cout << "Desired: " << std::endl;
    for (unsigned int i = 0; i < 6; i++){
      std::cout << v2.at(i) << "\t";
      if (i == 5) std::cout << std::endl;
    }

    // Check error: (which is fine)
    // vpColVector err = f.error(fd, 0);
    // std::cout << err << std::endl;

    task.addFeature(f, fd);

    vpMatrix check_interaction;
    check_interaction = task.computeInteractionMatrix ();
    std::cout << check_interaction << std::endl;

    vpSimulatorViper850 robot(true);
    robot.setVerbose(true);

    // Enlarge the default joint limits
    vpColVector qmin = robot.getJointMin();
    vpColVector qmax = robot.getJointMax();
    qmin[0] = -vpMath::rad(180);
    qmax[0] =  vpMath::rad(180);
    qmax[1] =  vpMath::rad(0);
    qmax[2] =  vpMath::rad(270);
    qmin[4] = -vpMath::rad(180);
    qmax[4] =  vpMath::rad(180);

    robot.setJointLimit(qmin, qmax);

    std::cout << "Robot joint limits: " << std::endl;
    for (unsigned int i=0; i< qmin.size(); i ++)
      std::cout << "Joint " << i << ": min " << vpMath::deg(qmin[i]) << " max " << vpMath::deg(qmax[i]) << " (deg)" << std::endl;

    robot.init(vpViper850::TOOL_PTGREY_FLEA2_CAMERA, vpCameraParameters::perspectiveProjWithoutDistortion);
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    robot.initScene(vpWireFrameSimulator::PLATE, vpWireFrameSimulator::D_STANDARD);
    robot.set_fMo(wMo);
    bool ret = true;

#if VISP_VERSION_INT > VP_VERSION_INT(2,7,0)
    ret =
    #endif

    robot.initialiseCameraRelativeToObject(cMo);

    if (ret == false)
      return 0; // Not able to set the position

    robot.setDesiredCameraPosition(cdMo);
    // We modify the default external camera position
    robot.setExternalCameraPosition(vpHomogeneousMatrix(vpTranslationVector(-0.4, 0.4, 2),
                                                        vpRotationMatrix(vpRxyzVector(M_PI/2,0,0))));

    vpImage<unsigned char> Iint(480, 640, 255);
#if defined(VISP_HAVE_X11)
    vpDisplayX displayInt(Iint, 700, 0, "Internal view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI displayInt(Iint, 700, 0, "Internal view");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV displayInt(Iint, 700, 0, "Internal view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpCameraParameters cam(840, 840, Iint.getWidth()/2, Iint.getHeight()/2);
    // Modify the camera parameters to match those used in the other simulations
    robot.setCameraParameters(cam);

    bool start = true;

    while(1) {

      cMo = robot.get_cMo ();

      for (unsigned int i = 0 ; i < 4 ; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }

      v1 = get_features (p);
      f.buildFrom (v1.at(0), v1.at(1), v1.at(2), v1.at(3), v1.at(4), v1.at(5));

      vpDisplay::display(Iint);
      robot.getInternalView(Iint);

      if (!start) {
        display_trajectory(Iint, point, cMo, cam);
        vpDisplay::displayText(Iint, 40, 120, "Click to stop the servo...", vpColor::red);
      }
      vpDisplay::flush(Iint);

      vpColVector v = task.computeControlLaw();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      vpColVector err;
      err = f.error (fd, 0);
      std::cout << "Error: " << std::endl << err << std::endl;

      // A click to exit
      if (vpDisplay::getClick(Iint, false)){
        v1.clear();
        v2.clear();
        break;
      }

      if (start) {
        start = false;
        v = 0;
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);
        vpDisplay::displayText(Iint, 40, 120, "Click to start the servo...", vpColor::blue);
        vpDisplay::flush(Iint);
        //vpDisplay::getClick(Iint);
      }

      vpTime::wait(1000*robot.getSamplingTime());
    }
    task.kill();
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
