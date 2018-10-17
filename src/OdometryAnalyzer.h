/*
 * OdometryAnalyzer.h
 *
 *  Created on: Aug 10, 2018
 *      Author: jasmin
 */

#ifndef ODOMETRYANALYZER_H_
#define ODOMETRYANALYZER_H_

#include <ros/ros.h>

#include "ThreadLocalize.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <string>


namespace ohm_tsd_slam {

class OdometryAnalyzer {
public:
	OdometryAnalyzer(obvious::TsdGrid* grid);
	virtual ~OdometryAnalyzer();

	void odomRescueInit();

  /**
   * odomRescueUpdate
   * updates odometry data if a new scan comes in
   */
  void odomRescueUpdate();

  /**
   * odomRescueCheck
   * check if slam transformation is plausible and overwrites T with odometry as transformation if not
   * @param T Transformation matrix to check and correct
   */
  void odomRescueCheck(obvious::Matrix& T);

  /**
   * obviouslyMatrix3x3ToTf
   * converts an 3x3 obvious matrix to a tf matrix
   * @param ob Obvious matrix to convert
   * @return transformed tf matrix
   */

private:

  obvious::Matrix tfToObviouslyMatrix3x3(const tf::Transform& tf);

  tf::TransformListener _tfListener;

  //Container for reading tfs
  tf::StampedTransform _tfReader;

  //ros tf frame ids
  std::string _tfFootprintFrameId;
  std::string _tfOdomFrameId;
  std::string _tfBaseFrameId;
  std::string _tfChildFrameId;

  //Transform from base footprint to laser
  tf::Transform _tfLaser;

  //Odom Transforms
  tf::Transform _tfOdomOld;
  tf::Transform _tfOdom;
  tf::Transform _tfRelativeOdom;

  //Laser time stamps
  ros::Time _stampLaser;
  ros::Time _stampLaserOld;

  //state of the actual odom tf
  bool _odomTfIsValid;

  //time to wait for synced odom tf
  ros::Duration _waitForOdomTf;

  obvious::TsdGrid& _grid;

  //ICP translation threshold
   double _trnsMax;
   double _trnsVelocityMax;

   //ICP rotation threshold
   double _rotMax;
   double _rotVelocityMax;

/*

  *
     * Pointer to main NodeHandle

    ros::NodeHandle* _nh;

  *
   * Pointer to mapping thread

  ThreadMapping& _mapper;

  *
   * namespace for all topics and services

  std::string _nameSpace;


  *
   * ICP rotation threshold

  double _rotMax;
  double _rotVelocityMax;

  *
    * Ros tf interface

  tf::TransformBroadcaster _tfBroadcaster;
  *
   * use odom rescue flag

  bool _useOdomRescue;

  *
   * Laser time stamps

  ros::Time _stampLaser;
  ros::Time _stampLaserOld;*/
};

} /* namespace ohm_tsd_slam */

#endif /* ODOMETRYANALYZER_H_ */
