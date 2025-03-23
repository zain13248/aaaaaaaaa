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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

import edu.wpi.first.math.MathUtil;
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
  private TalonFX elevatorMotor2;

  private TalonFXConfiguration eleMotorConfig1;
  private TalonFXConfiguration eleMotorConfig2;

  private CANcoder ElevatorEncoder;

    private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  double elevatorTarget = 0;
  //private final PositionVoltage m_voltagePosition = new PositionVoltage(0).withSlot(0); --OLD CODE

  private final MotionMagicVoltage m_voltagePosition = new MotionMagicVoltage(0).withSlot(0);
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {


    ElevatorEncoder = new CANcoder(6);

    elevatorMotor1 = new TalonFX(LEFT_ELEVATOR_MOTOR_1_ID);
    elevatorMotor2 = new TalonFX(RIGHT_ELEVATOR_MOTOR_2_ID);

    eleMotorConfig1 = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
      .withPeakForwardDutyCycle(0.75)
      .withPeakReverseDutyCycle(-0.75)
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive))
      .withFeedback(new FeedbackConfigs()
        .withFeedbackRemoteSensorID(ElevatorEncoder.getDeviceID()) 
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)); 

      eleMotorConfig2 = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
      .withPeakForwardDutyCycle(0.75)
      .withPeakReverseDutyCycle(-0.75)
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withFeedback(new FeedbackConfigs()
        .withFeedbackRemoteSensorID(ElevatorEncoder.getDeviceID()) 
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)); 


       eleMotorConfig1.Slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
       eleMotorConfig1.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
       eleMotorConfig1.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

       eleMotorConfig1.Slot0.kP = 6; // A position error of 2.5 rotations results in 12 V output
       eleMotorConfig1.Slot0.kI = 0.1; // no output for integrated error
       eleMotorConfig1.Slot0.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
       
       // set Motion Magic settings
       var motionMagicConfigs = eleMotorConfig1.MotionMagic;
       motionMagicConfigs.MotionMagicCruiseVelocity = 8.4; //4 // Target cruise velocity of 80 rps
       motionMagicConfigs.MotionMagicAcceleration = 170; // Target acceleration of 160 rps/s (0.5 seconds)
       motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)


    elevatorMotor1.getConfigurator().apply(eleMotorConfig1);
    elevatorMotor2.getConfigurator().apply(eleMotorConfig2);

    elevatorMotor2.setControl(new Follower(LEFT_ELEVATOR_MOTOR_1_ID, true));

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ElevatorPosition", getAbsolutePosition());
    SmartDashboard.putNumber("TESTING/elevatorTarget", elevatorMotor1.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("TESTING/elevatorClosedLoopError", Math.abs(elevatorTarget-elevatorMotor1.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("elevator velocity", getVelovityMotor());

  }

  // public void setPosition(double position){
  //   ElevatorEncoder.setPosition(position);
  // }


  public double getAbsolutePosition() {

    return ElevatorEncoder.getPosition().getValueAsDouble();
}


public double getVelovityMotor() {

  return elevatorMotor2.getVelocity().getValueAsDouble();
}


  public boolean isElevatorAtPosition(double targetPosition) {
    return Math.abs(getAbsolutePosition() - targetPosition) < 0.5;
}




public void setElevatorPosition(double setpoint) {

  // eleMotorConfig1.MotionMagic.MotionMagicCruiseVelocity = Velocity;
  // eleMotorConfig1.MotionMagic.MotionMagicAcceleration = acceleration;
  // eleMotorConfig1.MotionMagic.MotionMagicJerk = jerk;
  // elevatorMotor1.getConfigurator().apply(eleMotorConfig1);


  // elevatorMotor1.setControl(m_voltagePosition.withPosition(setpoint));


 elevatorMotor1.setControl(m_voltagePosition.withPosition(setpoint)); 

}

public void runPercent(double N_speed) {

    N_speed = MathUtil.clamp(N_speed, -6, 6); 

        leftOut.Output = N_speed;
        rightOut.Output = N_speed; 

    elevatorMotor1.setControl(leftOut);
    // elevatorMotor2.setControl(rightOut);
}


}



