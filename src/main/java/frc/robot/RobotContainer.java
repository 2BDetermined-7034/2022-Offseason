
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto.FiveBall;
import frc.robot.commands.Auto.OneMeter;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.climb.RunSolenoid;
import frc.robot.commands.climb.RunWinch;
import frc.robot.commands.indexer.EjectBot;
import frc.robot.commands.indexer.EjectTop;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.RunIntakeMotors;
import frc.robot.commands.intake.Solenoid;
import frc.robot.commands.intake.SolenoidToggle;
import frc.robot.commands.sensor.SensorOverride;
import frc.robot.commands.shooter.*;
import frc.robot.commands.vision.VisShoot;
import frc.robot.subsystems.*;
import frc.robot.commands.indexer.*;
//import frc.robot.commands.intake.*;
//import frc.robot.commands.shooter.*;
//import frc.robot.commands.vision.*;
//import frc.robot.commands.climb.*;
import frc.robot.controllers.*;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // Swerve
    private final SwerveDrive m_drivetrainSubsystem = new SwerveDrive();

    // Controllers
    private final XboxController m_controller = new XboxController(0);
    public final  XboxController m_operator = new XboxController(1);

    // Subsystems
    private final CargoIntake m_cargoIntake = new CargoIntake(); // Intake
    private final Indexer m_indexer = new Indexer(); // Indexer
    private final DigitalSensor m_analogSenseor = new DigitalSensor(); // Color sensor
    private final LimeLight m_limeLight = new LimeLight(); // Limelight
    private final Shooter m_shooter = new Shooter(); // Shooter
    public final Climber m_climber = new Climber(); //Climb

    private final TrollShot m_trollShot = new TrollShot(m_shooter, m_indexer, m_analogSenseor);
    //private final LaunchShot m_launch = new LaunchShot(m_shooter, m_indexer, m_analogSenseor);

    /* Shooter */
    //private final RunShooter m_runShooter = new RunShooter(m_shooter, m_indexer, () -> Constants.shooter.speed, m_analogSenseor);
    private final TestShoot m_testShoot = new TestShoot(m_shooter, m_indexer, m_analogSenseor);
    private final VisShoot m_shoot = new VisShoot(m_drivetrainSubsystem, m_limeLight, m_analogSenseor, m_indexer, m_shooter);
    /* Indexer */
    private final RunIndexer m_runIndexer = new RunIndexer(m_indexer, m_shooter, () -> Constants.Subsystem.Indexer.speed, m_analogSenseor, m_operator::getAButton);
    //private final SensorOverride m_sensorOverride = new SensorOverride(m_analogSenseor);

    /* Intake */
    private final RunIntakeMotors m_runIntake = new RunIntakeMotors(m_cargoIntake,  () -> Constants.Subsystem.Intake.speed, m_analogSenseor, m_operator::getAButton);
    private final SolenoidToggle m_intakeSolToggle = new SolenoidToggle(m_cargoIntake);
    //private final Solenoid m_intakeup = new Solenoid(m_cargoIntake, false);

    private final EjectBot m_ejectBot = new EjectBot(m_shooter, m_indexer, m_cargoIntake, m_analogSenseor);
    private final EjectTop m_ejectTop = new EjectTop(m_shooter, m_indexer, m_analogSenseor);

    /* Climber */
    public final RunSolenoid m_toggleClimbSolenoid = new RunSolenoid(m_climber);

    //public final AddCorrect addCorrect = new AddCorrect();
    //public final SubCorrect subCorrect = new SubCorrect();
    //public final ResetCorrect resetCorrect = new ResetCorrect();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Register
        m_cargoIntake.register();
        m_climber.register();
        m_shooter.register();
        m_analogSenseor.register();

        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrainSubsystem,
                () -> -modifyAxis(m_controller.getLeftY() * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> -modifyAxis(m_controller.getLeftX() * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> -modifyAxis(m_controller.getRightX() * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        ));

        // Configure the button bindings
        configureButtonBindings();
        SmartDashboard.putNumber("shooter", 0);
    }


    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        new Button(m_controller::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
        new Button(() -> m_controller.getRightTriggerAxis() > 0.1).whenHeld(m_runIntake);
        new Button(() -> m_controller.getRightTriggerAxis() > 0.1).whenHeld(m_runIndexer);
        new Button(() -> m_controller.getLeftTriggerAxis() > 0.1).whenHeld(m_shoot);
        //new Button(m_controller::getBButton).whenHeld(); do 6 volt shot
        new Button(m_controller::getYButton).whenPressed(m_intakeSolToggle);
        new Button(m_controller::getAButton).whenHeld(m_trollShot);
       // new Button(m_controller::getXButton) climb actuate


        //climbPad.getButton("B").toggleWhenPressed(m_toggleClimbSolenoid);
        //climbPad.getButton("Y").whenPressed(m_intakeup);
        new Button(m_operator::getStartButton).whenHeld(m_ejectTop);
        new Button(m_operator::getBackButton).whenHeld(m_ejectBot);

        new Button(m_operator::getBButton).whenPressed(m_toggleClimbSolenoid);
        new Button(m_operator::getLeftBumper).whenHeld(new RunWinch(m_climber, () -> -Constants.Subsystem.Climber.winchSpeed * 0.85, m_operator::getLeftTriggerAxis, m_operator::getRightTriggerAxis));
        new Button(m_operator::getRightBumper).whenHeld(new RunWinch(m_climber, () -> Constants.Subsystem.Climber.winchSpeed, m_operator::getLeftTriggerAxis, m_operator::getRightTriggerAxis));

    }


    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new FiveBall(m_drivetrainSubsystem);
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05) / 3;

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }


}
