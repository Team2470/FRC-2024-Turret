// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  private final TalonFX m_motor;
  private final CANcoder m_encoder;
  

  private enum ControlMode{
      kStop, kOpenLoop,

    };
  private ControlMode m_controlMode = ControlMode.kOpenLoop;
  private double m_demand = 0.0;
  

  /** Creates a new Turret. */
  public Turret() {

    




    //Turret encoder settings
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = -0.40673828125; //TODO Tune (You subtract the offset)
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    m_encoder = new CANcoder(31, "canivore");
    m_encoder.getConfigurator().apply(encoderConfig);

    //Turret motor settings
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO Tune
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.SensorToMechanismRatio = 1.0; //TODO Tune
    motorConfig.Feedback.RotorToSensorRatio = 700.0;

    //Soft limits so we don't find hard limits
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100.0; //TODO Tune
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -100.0; //TODO Tune

    m_motor = new TalonFX(31, "canivore");
    m_motor.getConfigurator().apply(motorConfig);

    m_motor.getPosition().setUpdateFrequency(50);
    m_motor.optimizeBusUtilization();

  }

  public Command openloopCommand(double voltage){
    return Commands.runEnd(
      ()-> {
        m_controlMode = ControlMode.kOpenLoop;
        m_demand = voltage;
      },
     ()-> {
        m_controlMode= ControlMode.kStop;
        m_demand = 0.0;
     },
      this
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Put stuff on SmartDashboard 
    SmartDashboard.putNumber("Turret Rotations", m_motor.getPosition().getValueAsDouble());

    switch (m_controlMode){
      case kOpenLoop:
        m_motor.setVoltage(m_demand);
        break;

      case kStop:

      default:
        m_motor.stopMotor();
        break;

    }
  }
}
