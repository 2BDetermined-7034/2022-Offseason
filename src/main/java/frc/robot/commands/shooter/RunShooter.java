// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.sensor.SensorOverride;
import frc.robot.subsystems.DigitalSensor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;


public class RunShooter extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DigitalSensor analogSensor;

    private final Shooter m_shooter;
    private final Indexer m_index;
    private final DoubleSupplier shooterSpeed;


    /**
     * Creates a new RunShooter.
     *
     * @param subsystem The subsystem used by this command.
     * @param speed DoubleSupplier for the motor.
     */
    public RunShooter(Shooter subsystem, Indexer indexer, DoubleSupplier speed, DigitalSensor sensor) {
        this.m_index = indexer;
        this.analogSensor = sensor;
        m_shooter = subsystem;
        shooterSpeed = speed;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If the timer is over 4 seconds, start the indexer, hypothetically should work
        new SensorOverride(analogSensor);

        m_index.setSpeed(shooterSpeed.getAsDouble());
        m_shooter.setSpeed(shooterSpeed.getAsDouble() * 2);// ??
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_index.setSpeed(0);
        m_shooter.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
