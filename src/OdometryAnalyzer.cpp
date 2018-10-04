/*
 * OdometryAnalyzer.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: jasmin
 */

#include "OdometryAnalyzer.h"



namespace ohm_tsd_slam {

OdometryAnalyzer::OdometryAnalyzer() {
	// TODO Auto-generated constructor stub

}

OdometryAnalyzer::~OdometryAnalyzer() {
	// TODO Auto-generated destructor stub
}

void OdometryAnalyzer::odomRescueInit()
{
  // get bf -> laser transform at init, aussuming its a static transform
  try
  {
    _tfListener.waitForTransform(_tfFootprintFrameId, _tfChildFrameId, ros::Time(0), ros::Duration(10.0));
    _tfListener.lookupTransform(_tfFootprintFrameId, _tfChildFrameId, ros::Time(0), _tfReader);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    exit(EXIT_FAILURE);
  }

  ROS_INFO_STREAM("Received static base_footprint to laser tf for odom rescue");
  _tfLaser = _tfReader;

  // get first map -> odom transform for initialization
  try
  {
    _tfListener.waitForTransform(_tfBaseFrameId, _tfOdomFrameId, ros::Time(0), ros::Duration(10.0));
    _tfListener.lookupTransform(_tfBaseFrameId, _tfOdomFrameId, ros::Time(0), _tfReader);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    exit(EXIT_FAILURE);
  }

  ROS_INFO_STREAM("Received first odom tf for initialization of odom rescue");
  // transform odom to laser frame
  _tfOdomOld = _tfReader;
}

void OdometryAnalyzer::odomRescueUpdate()
{
  // get new odom tf
  try
  {
    _tfListener.waitForTransform(_tfBaseFrameId, _tfOdomFrameId, _stampLaser, _waitForOdomTf);
    _tfListener.lookupTransform(_tfBaseFrameId, _tfOdomFrameId, _stampLaser, _tfReader);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    _odomTfIsValid = false;
  }

  _tfOdom = _tfReader;

  // calc diff odom -> odom(t-1) -> odom(t)
  _tfRelativeOdom = _tfOdomOld.inverse() * _tfOdom;

  // push state ahead
  _tfOdomOld = _tfOdom;

  _odomTfIsValid = true;
}

void OdometryAnalyzer::odomRescueCheck(obvious::Matrix& T_slam)
{
  // transform transformation from slam to odom e.g. form laser to base footprint system
  obvious::Matrix T_laserOnBaseFootprint = tfToObviouslyMatrix3x3(_tfLaser) * T_slam * tfToObviouslyMatrix3x3(_tfLaser).getInverse();

  // get dt
  ros::Duration dtRos = _stampLaser - _stampLaserOld;
  double dt = dtRos.sec + dtRos.nsec * 1e-9;

  // get velocities
  double dx = T_laserOnBaseFootprint(0,2);
  double dy = T_laserOnBaseFootprint(1,2);
  double dtrans = sqrt(pow(dx,2) + pow(dy,2));

  double drot = abs(asin(T_laserOnBaseFootprint(0,1))); // dont use acos here --> missing sign

  double vrot = drot / dt;
  double vtrans = dtrans / dt;

  // use odom instead of slam if slam translation is impossible for robot
  if(dtrans > _grid.getCellSize() * 2.0)
  {
    if(vrot > _rotVelocityMax || vtrans > _trnsVelocityMax)
    {
      ROS_INFO("-----ODOM-RECOVER-----");

      T_slam =
        tfToObviouslyMatrix3x3(_tfLaser).getInverse() *
        tfToObviouslyMatrix3x3(_tfRelativeOdom) *
        tfToObviouslyMatrix3x3(_tfLaser);
    }
  }
}


} /* namespace ohm_tsd_slam */
