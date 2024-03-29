// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DigitalSensor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;


public class TrollShot extends CommandBase {

    private final Shooter m_shooter;
    private final Indexer m_indexer;


    /**
     * Creates a new TrollShot.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TrollShot(Shooter subsystem, Indexer indexer) {
        this.m_indexer = indexer;
        m_shooter = subsystem;


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
        double visSpeed = 3;
        m_shooter.setSpeed(visSpeed);
        if (Math.abs(m_shooter.getVoltage() - visSpeed) <= Constants.Subsystem.Shooter.shooterVoltageRange) {
            //new SensorOverride(analogSensor);
            m_indexer.setSpeed(Constants.Subsystem.Indexer.speed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.setSpeed(0);
        m_shooter.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
