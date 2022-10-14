package frc.robot.commands.Drive;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.RunIntakeMotors;
import frc.robot.subsystems.*;

public class IntakePath extends ParallelDeadlineGroup {
    DigitalSensor sensor;

    public IntakePath(SwerveDrive m_drive, Indexer m_indexer, Shooter m_shooter, DigitalSensor m_sensor, CargoIntake m_cargoIntake, String path, boolean inverted) {
        super(new SwervePathGenerator(m_drive, path, inverted).getCommand(), new RunIndexer(m_indexer, m_shooter, () -> Constants.Subsystem.Indexer.speed, m_sensor, () ->false), new RunIntakeMotors(m_cargoIntake, () -> Constants.Subsystem.Intake.speed, m_sensor, () -> false));
        sensor = m_sensor;
    }

}