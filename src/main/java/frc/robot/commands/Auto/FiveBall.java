package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.SwervePathGenerator;
import frc.robot.subsystems.SwerveDrive;


public class FiveBall extends SequentialCommandGroup {

    public FiveBall(SwerveDrive m_swerveDrive) {

        addCommands(
            new SwervePathGenerator(m_swerveDrive, "5ball1", false).getCommand(),
            new SwervePathGenerator(m_swerveDrive, "5ball2", false).getCommand()
        );
    }
}