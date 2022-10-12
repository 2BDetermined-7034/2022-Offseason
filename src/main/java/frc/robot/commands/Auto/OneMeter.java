package frc.robot.commands.Auto;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drive.SwervePathGenerator;
import frc.robot.commands.intake.Solenoid;
import frc.robot.subsystems.*;


public class OneMeter extends SequentialCommandGroup {

    public OneMeter(SwerveDrive m_swerveDrive) {

        addCommands(
            new SwervePathGenerator(m_swerveDrive, "1mtest", false).getCommand()
        );
    }
}