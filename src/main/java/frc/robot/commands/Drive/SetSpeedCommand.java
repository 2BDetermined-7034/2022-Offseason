package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;


public class SetSpeedCommand extends CommandBase {
    private final SwerveDrive m_swerveDrive;

    private final double x;
    private final double y;
    private final double r;

    public SetSpeedCommand(SwerveDrive swerveDrive, double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this.m_swerveDrive = swerveDrive;
        this.x = vxMetersPerSecond;
        this.y = vyMetersPerSecond;
        this.r = omegaRadiansPerSecond;
        addRequirements(this.m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_swerveDrive.drive(new ChassisSpeeds(x, y, r));
    }


    @Override
    public boolean isFinished() {
        return true;
    }

}
