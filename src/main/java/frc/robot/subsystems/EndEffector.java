package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_TOLERANCE;
import static frc.robot.Constants.ElevatorConstants.LEFT_ELEVATOR_MOTOR_1_ID;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.CANVenom.ControlMode;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffector extends SubsystemBase {
    
    private final TimeOfFlight sensor;
    private static final int SENSOR_ID = 1; 
    private static final double PWFdistance = 135; //DETECTS UP TO 20CM
    private final TalonFX EndEffectorMotorLeft;
    private final TalonFX EndEffectorMotorRight;
    private final TalonFX EndEffectorIntake;
    private final DutyCycleOut out = new DutyCycleOut(0);
  private final PositionVoltage m_intakePosition = new PositionVoltage(0).withSlot(0);

    private TalonFXConfiguration MotorConfig1;
    private TalonFXConfiguration MotorConfig2;

    private TalonFXConfiguration MotorConfigIntake;

    double elevatorTarget = 0;
      private CANcoder PivotEncoder;

  private final MotionMagicVoltage m_voltagePosition = new MotionMagicVoltage(0).withSlot(0);

     

    public EndEffector() {
      PivotEncoder = new CANcoder(10); 

    EndEffectorMotorLeft = new TalonFX(10);
    EndEffectorMotorRight = new TalonFX(13);

    EndEffectorIntake = new TalonFX(12); //28

    MotorConfig1 = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive))
      .withFeedback(new FeedbackConfigs()
      .withFeedbackRemoteSensorID(PivotEncoder.getDeviceID())
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));

    MotorConfig2 = new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive))
    .withFeedback(new FeedbackConfigs()
    .withFeedbackRemoteSensorID(PivotEncoder.getDeviceID()) 
    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));

    MotorConfigIntake = new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Brake)
    .withInverted(InvertedValue.Clockwise_Positive));

    MotorConfigIntake.Slot0.kP = 1; // A position error of 2.5 rotations results in 12 V output
    MotorConfigIntake.Slot0.kI = 0; // no output for integrated error
    MotorConfigIntake.Slot0.kD = 0; 

    EndEffectorMotorLeft.getConfigurator().apply(MotorConfig1);
    EndEffectorMotorRight.getConfigurator().apply(MotorConfig2);




    MotorConfig1.Slot0.kS = 0; // Add 0.25 V output to overcome static friction
    MotorConfig1.Slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    MotorConfig1.Slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output

    MotorConfig1.Slot0.kP = 27; // A position error of 2.5 rotations results in 12 V output
    MotorConfig1.Slot0.kI = 5; // no output for integrated error
    MotorConfig1.Slot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = MotorConfig1.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.47; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)





      EndEffectorIntake.getConfigurator().apply(MotorConfigIntake);
      EndEffectorMotorLeft.getConfigurator().apply(MotorConfig1);
      EndEffectorMotorRight.getConfigurator().apply(MotorConfig2);
      EndEffectorMotorRight.setControl(new Follower(13, true));

      // EndEffectorMotorLeft.setControl(new Follower(10, true));

        sensor = new TimeOfFlight(SENSOR_ID);
        sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);


    }

public void runPercent(double N_speed) {

    N_speed = MathUtil.clamp(N_speed, -6, 6); 

        out.Output = N_speed;


        EndEffectorMotorLeft.setControl(out);
    
}

    @Override
  public void periodic() {

    SmartDashboard.putBoolean("EndEffectorSensor", isChoralValid());

    SmartDashboard.putBoolean("EndEffectorSensorNOT", isChoralInvalid());
    SmartDashboard.putNumber("PivotPosition", getAbsolutePosition());
    SmartDashboard.putNumber("range ", sensor.getRange());

    SmartDashboard.putNumber("TESTING/elevatorTarget", elevatorTarget);

    SmartDashboard.putNumber("endeffector velocity", getVelovityMotor());
    SmartDashboard.putNumber("endeffector current", EndEffectorIntake.getStatorCurrent().getValueAsDouble());


  }


  public double getAbsolutePosition() {
    return PivotEncoder.getPosition().getValueAsDouble();
}


  public boolean isEndEffectorAtPosition(double targetPosition) {
    return Math.abs(getAbsolutePosition() - targetPosition) < 0.5;
}

public boolean isCurrentOverThreshold() {
  return EndEffectorIntake.getStatorCurrent().getValueAsDouble() > 20;
}



public void setEndEffectorPosition(double setpoint) { // double Velocity, double acceleration, double jerk

  // MotorConfig1.MotionMagic.MotionMagicCruiseVelocity = Velocity;
  // MotorConfig1.MotionMagic.MotionMagicAcceleration = acceleration;
  // MotorConfig1.MotionMagic.MotionMagicJerk = jerk;
  // EndEffectorMotorLeft.getConfigurator().apply(MotorConfig1);

  EndEffectorMotorLeft.setControl(m_voltagePosition.withPosition(setpoint));



}


public void runIntake(double speed) {
    EndEffectorIntake.set(speed);
}




    public boolean isChoralValid() {
        if (sensor.isRangeValid()) {
            double distance = sensor.getRange();
            return (distance <= PWFdistance);
        } else {
            return false;
        }
    }

    public double getIntakePosition() {
      return EndEffectorIntake.getPosition().getValueAsDouble(); // Correct method for TalonFX
  }

  public double getVelovityMotor() {

    return EndEffectorMotorLeft.getVelocity().getValueAsDouble();
  }
  

// Converts rotations to encoder units (if using integrated encoder)

public void runMotorAfterChoralValid() {
    if (isChoralValid()) {
        double currentPosition = getIntakePosition(); // Get current position
        double targetPosition = currentPosition - (29.6); // Convert 10 rotations to encoder ticks

        EndEffectorIntake.setControl(m_intakePosition.withPosition((targetPosition))); // Set TalonFX to position mode
    }
}

  

    public boolean isChoralInvalid() {
      return !isChoralValid();
  }
  




    public void stopIntake() {
      EndEffectorIntake.set(0);

    }


}


