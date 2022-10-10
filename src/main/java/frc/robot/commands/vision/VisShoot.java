package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.sensor.SensorOverride;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class VisShoot extends CommandBase {

    private final SwerveDrive m_swerve;
    private final LimeLight m_ll;

    private BooleanSupplier m_vis;
    private BooleanSupplier m_interrupt;

    private final DigitalSensor analogSensor;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    private boolean tapeDetected;

    private int tapeTimer;

    private double errorX;

    public VisShoot(SwerveDrive swerveDrive, LimeLight ll, DigitalSensor analogSensor, Indexer indexer, Shooter shooter, BooleanSupplier useVision, BooleanSupplier interrupt) {
        m_swerve = swerveDrive;
        m_ll = ll;
        m_vis = useVision;
        m_interrupt = interrupt;

        this.analogSensor = analogSensor;
        m_indexer = indexer;
        m_shooter = shooter;

        addRequirements(m_swerve);
        addRequirements(ll);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        tapeDetected = false;
        tapeTimer = 0;
        errorX = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_ll.setLights(true);
        boolean vis = m_vis.getAsBoolean();
        tapeDetected = m_ll.getDetected();

        if(tapeDetected) {
            tapeTimer++;
        }
        else {
            tapeTimer = 0;
        }

        if (tapeTimer >= Constants.Subsystem.Vision.Vis_TimerConfidence && vis) {
            double visX = m_ll.getXAngle() + Constants.Subsystem.Vision.VisX_Offset;
            //add .6 to errorX to help reach actual target
            errorX = visX > 0 ? visX + .9 : visX - .9;
        }
        if (tapeDetected) {
            m_swerve.drive(new ChassisSpeeds(0, 0, -errorX / Constants.Subsystem.Vision.pGain));
        }

        double llY = m_ll.getYAngle();
        SmartDashboard.putNumber("lly", llY);
        double visSpeed;


        visSpeed = -1 * (5.1 + (.00695 * llY) + (.00146 * Math.pow(llY, 2)) + (.00034 * Math.pow(llY, 3)) + SmartDashboard.getNumber("ad", 0));

        m_shooter.setSpeed(visSpeed);
        if (Math.abs(m_shooter.getVoltage() - visSpeed) <= Constants.Subsystem.Shooter.shooterRange && tapeDetected) {
            new SensorOverride(analogSensor);
            m_indexer.setSpeed(Constants.Subsystem.Indexer.speed);
        }



    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return m_interrupt.getAsBoolean();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_shooter.setSpeed(Constants.Subsystem.Shooter.passiveFullSpeed);
        m_indexer.setSpeed(0);
        m_ll.setLights(false);
    }
}