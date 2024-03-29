// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter subsystem */

    private final CANSparkMax m_motor;
    private final CANSparkMax m_motor2;
    private final MotorControllerGroup m_controlGroup;
    private double m_speeds;

    public Shooter() {
        m_motor = new CANSparkMax(Constants.Subsystem.Shooter.leftShooterMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor2 = new CANSparkMax(Constants.Subsystem.Shooter.rightShooterMotor, CANSparkMaxLowLevel.MotorType.kBrushless);


        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);
        m_motor.setInverted(false);
        m_motor2.setInverted(true);

        m_controlGroup = new MotorControllerGroup(m_motor, m_motor2);
        m_speeds = 0;
        SmartDashboard.putNumber("Shooter Speed", 0);
        SmartDashboard.putNumber("ad", 0);
    }

  @Override
  public void periodic() {

        SmartDashboard.putNumber("Shooter Vis Speed", m_speeds);
        SmartDashboard.putNumber("Shooter Voltage", getVoltage());
        m_motor.setVoltage(m_speeds);
        m_motor2.setVoltage(m_speeds);
  }

  /**
   * Method for command
   * @param speed to be taken from Vision
   */
  public void setSpeed(double speed) {
      m_speeds = speed;
  }

  public double getVoltage() {
      return m_motor.getAppliedOutput() * m_motor.getBusVoltage();
  }


  public void debug() {
      SmartDashboard.putNumber("Shooter Speed", m_speeds);
  }
}
