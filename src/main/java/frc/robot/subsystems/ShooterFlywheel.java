// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterFlywheel extends SubsystemBase {

  private enum ControlMode {
    kOpenLoop, kPID
  }

  private ControlMode m_controlMode = ControlMode.kOpenLoop;

  private final PIDController m_pidController = new PIDController( 0.001, 0, 0);
  private final TalonFX m_motor;
  private final VoltageOut m_motorRequest = new VoltageOut(0);
  private double m_demand;
  

  

  /** Creates a new ShooterFlywheel. */
  public ShooterFlywheel(int motorID, boolean isInverted) {
    m_motor = new TalonFX(motorID);

 
	//m_motor.setInverted(isLeft);
  m_motor.setInverted(isInverted);
	m_motor.getPosition().setUpdateFrequency(50);
	m_motor.optimizeBusUtilization();

  }

  public void setVoltage(double voltage){
  
    m_motorRequest.withOutput(voltage);
    m_motor.setControl(m_motorRequest);
  }

  public double getRPM(){
    return m_motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
  SmartDashboard.putNumber("demand", m_demand);
  SmartDashboard.putNumber("FlywheelRPM", getRPM());


    double outputVoltage = 0;
  switch (m_controlMode) {
	case kOpenLoop:
		// Do openloop stuff here
		outputVoltage = m_demand;
		break;

	case kPID:
		m_pidController.setP(SmartDashboard.getNumber("kP", 0.001));
		m_pidController.setI(SmartDashboard.getNumber("kI", 0));
		m_pidController.setD(SmartDashboard.getNumber("kD", 0));
		double kF = SmartDashboard.getNumber("kF",  0.00016666666);

		// Do PID stuff
		outputVoltage = kF * m_demand + m_pidController.calculate(getRPM(), m_demand);

		break;
	  default:
		// What happened!?
		break;
	}



    // This method will be called once per scheduler run
  }

  public void setPIDSetpoint(double rpm) {
    m_controlMode = ControlMode.kPID;
    m_demand = rpm;
  }

  public Command pidCommand(DoubleSupplier rpmSupplier){
	  return Commands.runEnd(
	  () -> this.setPIDSetpoint(rpmSupplier.getAsDouble()), this::stop, this);
}


  public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {


  	// Inline construction of command goes here.
  	// Subsystem::RunOnce implicitly requires `this` subsystem.
	  return Commands.runEnd(
	  	() -> this.setVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);

  }

  public Command openLoopCommand(double OutputVoltage) {
	  return openLoopCommand(()-> OutputVoltage);
  }

  public Command pidCommand(double rpm){
	  return pidCommand(() -> rpm);
  }

  public void stop() {
    setVoltage(0);
  }
}
