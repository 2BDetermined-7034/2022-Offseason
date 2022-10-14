package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.IntakePath;
import frc.robot.commands.Drive.SwervePathGenerator;
import frc.robot.commands.vision.TimedVisShoot;
import frc.robot.subsystems.*;


public class FiveBall extends SequentialCommandGroup {

    public FiveBall(SwerveDrive m_swerveDrive, LimeLight m_ll, Indexer m_indexer, Shooter m_shooter, DigitalSensor m_sensor, CargoIntake m_intake) {

        addCommands(
            new IntakePath(m_swerveDrive, m_indexer, m_shooter, m_sensor, m_intake, "5ball1", false),
            new TimedVisShoot(m_swerveDrive, m_ll, m_sensor, m_indexer, m_shooter, 2.5),
            new IntakePath(m_swerveDrive, m_indexer, m_shooter, m_sensor, m_intake, "5ball2", false),
            new TimedVisShoot(m_swerveDrive, m_ll, m_sensor, m_indexer, m_shooter, 2.5)
        );
    }
}