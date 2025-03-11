// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;



import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import static frc.robot.Constants.ElevatorConstants.*;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFXConfiguration eleMotorConfig;
  private double zeroPoint;
  private CANcoder ElevatorEncoder;


  double elevatorTarget = 0;
  private final PositionVoltage m_voltagePosition = new PositionVoltage(0).withSlot(0);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {


    ElevatorEncoder = new CANcoder(9);

    elevatorMotor1 = new TalonFX(ELEVATOR_MOTOR_1_ID);
    eleMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
      .withPeakForwardDutyCycle(0.5)
      .withPeakReverseDutyCycle(-0.1)
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withFeedback(new FeedbackConfigs()
        .withFeedbackRemoteSensorID(ElevatorEncoder.getDeviceID()) 
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)); 



       eleMotorConfig.Slot0.kP = 4;
       eleMotorConfig.Slot0.kI = 1.;
       eleMotorConfig.Slot0.kD = 0;
       eleMotorConfig.Slot0.kG = 0.3;

    
    elevatorMotor1.getConfigurator().apply(eleMotorConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ElevatorPosition", getAbsolutePosition());
    SmartDashboard.putNumber("TESTING/elevatorTarget", elevatorTarget);
    SmartDashboard.putNumber("TESTING/elevatorClosedLoopError", Math.abs(elevatorTarget-elevatorMotor1.getPosition().getValueAsDouble()));
  }

  public double getAbsolutePosition() {
    return ElevatorEncoder.getPosition().getValueAsDouble();
}
  public boolean isElevatorAtPosition(double targetPosition) {
    return Math.abs(getAbsolutePosition() - targetPosition) < ELEVATOR_TOLERANCE;
}




  public void setElevatorPosition(double setpoint) {

    elevatorMotor1.setControl(m_voltagePosition.withPosition(setpoint));

  }
}



