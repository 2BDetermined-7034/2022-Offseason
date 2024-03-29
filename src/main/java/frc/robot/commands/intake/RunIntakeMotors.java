package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.DigitalSensor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class RunIntakeMotors extends CommandBase {
    private final CargoIntake m_intake;
    private final DoubleSupplier fowS;
    private final DigitalSensor m_analog;
    private final BooleanSupplier m_stop;
    /**
     *
     * @param intakeMotor The intake motor
     * @param speed The speed you want the intake to move at.
     */

    public RunIntakeMotors(CargoIntake intakeMotor, DoubleSupplier speed, DigitalSensor colorSensor, BooleanSupplier stop){
        this.fowS = speed;
        this.m_intake = intakeMotor;
        this.m_stop = stop;
        this.m_analog = colorSensor;
        addRequirements(intakeMotor);
    }

    @Override
    public void initialize() {
        if (!m_analog.sensorBoolean1_2()){
            m_intake.setSolenoid(false);
        }
    }

    @Override
    public void execute() {

        double speed = fowS.getAsDouble();
        if (m_stop.getAsBoolean()){
            m_intake.setSpeed(speed);
        } else {
            m_intake.setSpeed(Constants.Subsystem.Intake.stopSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return m_analog.sensorBoolean1_2();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setSolenoid(true);
        m_intake.setSpeed(0);
    }
}