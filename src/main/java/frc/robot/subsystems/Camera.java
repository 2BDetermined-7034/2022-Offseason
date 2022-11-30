// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.camera;
import org.photonvision.PhotonCamera;

public class Camera extends SubsystemBase {

  PhotonCamera camera;

  SwerveDrivePoseEstimator m_poseEstimator;







  /** Creates a new Camera. */
  public Camera(PhotonCamera camera, SwerveDrivePoseEstimator m_poseEstimator) {
    this.camera = camera;
    this.m_poseEstimator = m_poseEstimator;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void update(
            DifferentialDriveWheelSpeeds actWheelSpeeds, double leftDist, double rightDist) {
        m_poseEstimator.update(gyro.getRotation2d(), actWheelSpeeds, leftDist, rightDist);

        var res = camera.getLatestResult();
        if (res.hasTargets()) {
            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
            Transform3d camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
            Pose2d camPose = Constants.camera.targetPoses[0].transformBy(camToTargetTrans.inverse());
            m_poseEstimator.addVisionMeasurement(
                    camPose.transformBy(Constants.kCameraToRobot), imageCaptureTime);
        }
    }

  public Pose2d getFieldPose() {

    var result = camera.getLatestResult();




    

    return null;
  }


}
