/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH; //SCOTTY

  cheetah._bodyMass = 19.31;
  cheetah._bodyLength = 0.4;
  cheetah._bodyWidth = 0.25;
  cheetah._bodyHeight = 0.15;
  cheetah._abadGearRatio = 9;
  cheetah._hipGearRatio = 9;
  cheetah._kneeGearRatio = 9;
  cheetah._abadLinkLength = 0.07;
  cheetah._hipLinkLength = 0.3175;
  cheetah._kneeLinkY_offset = 0.09;
  cheetah._kneeLinkLength = 0.3375;
  cheetah._maxLegLength = 0.655;


  cheetah._motorTauMax = 3.f;
  cheetah._batteryV = 24;
  cheetah._motorKT = 0.198;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.195;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;
  //cheetah._jointDamping = .0;
  //cheetah._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 0.01002, 0, 0, 0, 0.01002, 0, 0, 0, 0.01002;
//   rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 0.00,  0.00,  0.00, 0.00,  0.01,  0.00, 0.00,  0.00,  0.00;
  // abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0.06, 0.00, 0.00);  // FRONT LEFT
  SpatialInertia<T> abadInertia(1.03, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 0.02,  0.00,  0.00, 0.00,  0.02, -0.01, 0.00, -0.01,  0.01;
  // hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0.00, 0.06, -0.04);
  SpatialInertia<T> hipInertia(1.74, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 0.01,  0.00,  0.00, 0.00,  0.01,  0.00, 0.00,  0.00,  0.00;
  // kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0.01, 0.00, -0.11);
  SpatialInertia<T> kneeInertia(0.54, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.384 , rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.384 , rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 0.05,  0.00,  0.00, 0.00,  0.24,  0.00, 0.00,  0.00,  0.27;
  // bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0.00, 0.00, 0.02);
  SpatialInertia<T> bodyInertia(6.03 , bodyCOM,
                                bodyRotationalInertia); //only body link mass

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  cheetah._abadRotorLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
