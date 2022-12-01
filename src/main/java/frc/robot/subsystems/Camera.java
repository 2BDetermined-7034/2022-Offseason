// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;

public class Camera extends SubsystemBase {

  PhotonCamera camera;

  public SwerveDrivePoseEstimator poseEstimator;







  /** Creates a new Camera. */
  public Camera() {
    this.camera = new PhotonCamera( Constants.camera.cameraName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void update(SwerveDrive swerveDrive, SwerveDrivePoseEstimator poseEstimator) {
        poseEstimator.update(swerveDrive.getGyroscopeRotation(),swerveDrive.m_states); //FIXME pass in robot rotation

        var res = camera.getLatestResult();
        if (res.hasTargets()) {
            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
            Transform3d camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
            Pose3d camPose = Constants.camera.targetPoses[0].transformBy(camToTargetTrans.inverse()); //Can weigh multiple poses later
            poseEstimator.addVisionMeasurement(
                    camPose.toPose2d(), imageCaptureTime); //TODO transform camPose with camToRobot
        }
    }

  public Pose2d getEstimatedPose() {

    return poseEstimator.getEstimatedPosition();


  }


}
