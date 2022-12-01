// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.HashMap;

import org.photonvision.PhotonCamera;

public class Camera extends SubsystemBase {

  private PhotonCamera camera;



  public static final double[][] tagXYZ = {
    {Units.inchesToMeters(300),Units.inchesToMeters(324-72-12), Units.inchesToMeters(30)},
    {Units.inchesToMeters(300),Units.inchesToMeters(324-72-12-84-12), Units.inchesToMeters(30)}
  };
        // We can only place tags on score towers facing us, meaning we only need 2 for odometry on our side of the field

  public static final HashMap<Integer, Pose3d> targetMap;
  /** [FiducialId, Pose] */
  static{
    targetMap = new HashMap<>();
    targetMap.put(0, new Pose3d(
      tagXYZ[0][0],
      tagXYZ[0][1],
      tagXYZ[0][2],
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180))));
    targetMap.put(1, new Pose3d(
      tagXYZ[1][0], 
      tagXYZ[1][1], 
      tagXYZ[1][2], 
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180))));
  }

  /** Creates a new Camera. */
  public Camera() {
    this.camera = new PhotonCamera( Constants.camera.cameraName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void update(SwerveDrive swerveDrive, SwerveDrivePoseEstimator poseEstimator) {
        poseEstimator.update(swerveDrive.getGyroscopeRotation(),swerveDrive.m_states);

        var res = camera.getLatestResult();
        if (res.hasTargets()) {
            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
            Transform3d camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
            int targetID = res.getBestTarget().getFiducialId();
            Pose3d camPose = targetMap.get(targetID).transformBy(camToTargetTrans.inverse()); //Can weigh multiple poses later
            poseEstimator.addVisionMeasurement(
                    camPose.toPose2d(), imageCaptureTime); //TODO transform camPose with camToRobot
        }
    }



}
