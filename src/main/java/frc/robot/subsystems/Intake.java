// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX m_motor;
  /** Creates a new ok. */
  public Intake() {
    m_motor = new TalonFX(10);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public void in() {
    m_motor.setVoltage(6);
  }

  public void out() {
    m_motor.setVoltage(-6);
  }

  public Command inCommand() {
    return Commands.runEnd(()->in(), ()->stop(), this);
  }

  public Command outCommand() {
    return Commands.runEnd(()->out(), ()->stop(), this);
  }
  
}

  