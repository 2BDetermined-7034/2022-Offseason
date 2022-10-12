package frc.robot.commands.Drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class SwervePathGenerator {

    PPSwerveControllerCommand m_follower;
    SwerveDrive m_swerveDrive;
    PathPlannerTrajectory m_trajectory;


    public SwervePathGenerator(SwerveDrive swerveDrive, String pathName, boolean reversed){
        this.m_swerveDrive = swerveDrive;

        try {
            m_trajectory = PathPlanner.loadPath(pathName, m_swerveDrive.getMaxVelocityMetersPerSecond(), m_swerveDrive.getMaxAcceleration(), reversed);
        } catch (TrajectoryParameterizer.TrajectoryGenerationException exception) {
            DriverStation.reportError("Failed to load trajectory", false);
        }

        var thetaController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(m_swerveDrive.getMaxAngularVelocity(), m_swerveDrive.getMaxAngularVelocity() * 2.0));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_follower = new PPSwerveControllerCommand(
                m_trajectory,
                m_swerveDrive::getPosition,
                m_swerveDrive.getKinematics(),
                new PIDController(Constants.Auto.kP, 0, 0),
                new PIDController(Constants.Auto.kP, 0, 0),
                thetaController,
                m_swerveDrive::setModuleStates,
                m_swerveDrive
        );

    }

    public SequentialCommandGroup getCommand(){
        return new SetPositionCommand(m_swerveDrive, m_trajectory.getInitialPose()).andThen(m_follower).andThen(new SetSpeedCommand(m_swerveDrive,  0, 0, 0));
    }
}
