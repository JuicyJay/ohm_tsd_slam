/*
 * OdometryAnalyzer.h
 *
 *  Created on: Aug 10, 2018
 *      Author: jasmin
 */

#ifndef ODOMETRYANALYZER_H_
#define ODOMETRYANALYZER_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <string>


namespace ohm_tsd_slam {

class OdometryAnalyzer {
public:
	OdometryAnalyzer();
	virtual ~OdometryAnalyzer();

private:
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

  /**
    * Ros tf interface
    */
  tf::TransformBroadcaster _tfBroadcaster;
  tf::TransformListener _tfListener;


  /**
     * Odom Transforms
     */
  tf::Transform _tfOdomOld;
  tf::Transform _tfOdom;
  tf::Transform _tfRelativeOdom;

  /**
   * ros tf frame ids
   */
  std::string _tfFootprintFrameId;
  std::string _tfOdomFrameId;
  std::string _tfBaseFrameId;
  std::string _tfChildFrameId;

  /**
   * use odom rescue flag
   */
  bool _useOdomRescue;

  /**
   * state of the actual odom tf
   */
  bool _odomTfIsValid;

  /**
   * time to wait for synced odom tf
   */
  ros::Duration _waitForOdomTf;

  /**
   * Laser time stamps
   */
  ros::Time _stampLaser;
  ros::Time _stampLaserOld;
};

} /* namespace ohm_tsd_slam */

#endif /* ODOMETRYANALYZER_H_ */
