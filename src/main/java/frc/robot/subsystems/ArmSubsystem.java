package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import static frc.robot.Constants.ElevatorConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX ArmLeftMotor;
  private TalonFX ArmRightMotor;

  private TalonFXConfiguration ArmConfig;
  private double zeroPoint;
  private CANcoder ArmEncoder;

  private final PositionVoltage m_voltagePosition = new PositionVoltage(0).withSlot(0);

  double elevatorTarget = 0;

  /** Creates a new ElevatorSubsystem. */
  public ArmSubsystem() {
    ArmEncoder = new CANcoder(7); 
    ArmLeftMotor = new TalonFX(13);
    ArmRightMotor = new TalonFX(10);
    ArmConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive))
      .withFeedback(new FeedbackConfigs()
      .withFeedbackRemoteSensorID(ArmEncoder.getDeviceID()) 
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
      


       ArmConfig.Slot0.kP = 15;
       ArmConfig.Slot0.kI =1;
       ArmConfig.Slot0.kD = 0;
    
    ArmLeftMotor.getConfigurator().apply(ArmConfig);
    ArmRightMotor.getConfigurator().apply(ArmConfig);
    ArmRightMotor.setControl(new Follower(10, true));



  }


  @Override
  public void periodic() {

    SmartDashboard.putNumber("TESTING/elevatorTarget", elevatorTarget);
    SmartDashboard.putNumber("ArmPosition", getAbsolutePosition());

    SmartDashboard.putNumber("TESTING/elevatorClosedLoopError", Math.abs(elevatorTarget-ArmLeftMotor.getPosition().getValueAsDouble()));
  }
  public double getAbsolutePosition() {
    return ArmEncoder.getPosition().getValueAsDouble();
}
  public boolean isArmAtPosition(double targetPosition) {
    return Math.abs(getAbsolutePosition() - targetPosition) < ELEVATOR_TOLERANCE;
}



public void setArmPosition(double setpoint) {
  // System.out.println("ARMMMMMMMM");
  ArmLeftMotor.setControl(m_voltagePosition.withPosition(setpoint));
    // setpoint =Math.min( Math.max(setpoint , Constants.Arm.CANAmp ) , Constants.Arm.CANRest);
//     pid1.setReference(setpoint, ControlType.kPosition);
//     pid2.setReference(setpoint, ControlType.kPosition);

}
}
