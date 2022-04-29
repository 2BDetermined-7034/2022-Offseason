package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;


public class SetPositionCommand extends CommandBase {
    private final SwerveDrive m_swerveDrive;
    private final Pose2d m_position;

    public SetPositionCommand(SwerveDrive swerveDrive, Pose2d position) {
        this.m_swerveDrive = swerveDrive;
        this.m_position = position;
        addRequirements(this.m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_swerveDrive.setPosition(m_position);
    }


    @Override
    public boolean isFinished() {
        return true;
    }

}
