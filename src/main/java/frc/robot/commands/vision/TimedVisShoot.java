package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.sensor.SensorOverride;
import frc.robot.subsystems.*;


public class TimedVisShoot extends CommandBase {

    private final SwerveDrive m_swerve;
    private final LimeLight m_ll;
    private final DigitalSensor analogSensor;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    private final PIDController rotatePID;

    private boolean tapeDetected;
    private final Timer timer = new Timer();
    private final double totalTime;

    private int tapeTimer;

    public TimedVisShoot(SwerveDrive swerveDrive, LimeLight ll, DigitalSensor analogSensor, Indexer indexer, Shooter shooter, double time) {
        m_swerve = swerveDrive;
        m_ll = ll;

        this.analogSensor = analogSensor;
        m_indexer = indexer;
        m_shooter = shooter;

        rotatePID = new PIDController(Constants.Subsystem.Vision.pGain, 0, 0);
        totalTime = time;

        addRequirements(m_swerve);
        addRequirements(ll);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        tapeDetected = false;
        tapeTimer = 0;
        timer.reset();
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_ll.setLights(true);
        tapeDetected = m_ll.getDetected();

        if(tapeDetected) {
            tapeTimer++;
        }
        else {
            tapeTimer = 0;
        }

        if (tapeTimer >= Constants.Subsystem.Vision.Vis_TimerConfidence) {
            double llx = m_ll.getXAngle();
            double llY = m_ll.getYAngle();
            SmartDashboard.putNumber("llx", llx);
            SmartDashboard.putNumber("lly", llY);
            double visSpeed = (5.084 + (-0.034 * llY) + (-0.001 * Math.pow(llY, 2)) + (0.000377 * Math.pow(llY, 3)) + SmartDashboard.getNumber("ad", 0));
            double zSpeed = rotatePID.calculate(llx, 0);
            SmartDashboard.putNumber("shooter speed", visSpeed);
            m_swerve.drive(new ChassisSpeeds(0, 0, zSpeed));
            m_shooter.setSpeed(visSpeed);

            if (Math.abs(m_shooter.getVoltage() - visSpeed) <= Constants.Subsystem.Shooter.shooterVoltageRange) {
                if (Math.abs(llx) <= Constants.Subsystem.Shooter.shooterOffsetRange){
                    new SensorOverride(analogSensor);
                    m_indexer.setSpeed(Constants.Subsystem.Indexer.speed);
                }
            }
        } else {
            m_swerve.drive(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() > totalTime;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_shooter.setSpeed(Constants.Subsystem.Shooter.passiveFullSpeed);
        m_indexer.setSpeed(0);
        m_ll.setLights(false);
    }
}