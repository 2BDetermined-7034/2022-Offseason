// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrive;

public class CameraCommand extends CommandBase {

  public Camera camera;
  public SwerveDrivePoseEstimator poseEstimator;
  private SwerveDrive swerveDrive;

  /** Creates a new CameraCommand. */
  public CameraCommand(Camera camera, SwerveDrive swerveDrive) {
    this.camera = camera;
    this.swerveDrive = swerveDrive;

    poseEstimator = new SwerveDrivePoseEstimator(swerveDrive.getGyroscopeRotation(), new Pose2d(), 
    swerveDrive.m_kinematics,
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02), // [x, y, theta] State Standard Deviations Increase to trust less
    new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01), // Encoder Gyro Standard Deviations [theta]
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01) // Vision Standard Deviations [x, y, theta]

    );


    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.update(swerveDrive, poseEstimator);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
