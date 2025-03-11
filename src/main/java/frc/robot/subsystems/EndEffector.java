package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_TOLERANCE;

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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffector extends SubsystemBase {
    
    private final DutyCycleOut motorOut = new DutyCycleOut(0);
    private final TimeOfFlight sensor;
    private static final int SENSOR_ID = 1; 
    private static final double PWFdistance = 200; //DETECTS UP TO 20CM
    private final TalonFX EndEffectorMotor;
    private final TalonFX EndEffectorIntake;

    private TalonFXConfiguration MotorConfig;
    private TalonFXConfiguration MotorConfigIntake;

    double elevatorTarget = 0;
      private CANcoder PivotEncoder;

  private final PositionVoltage m_voltagePosition = new PositionVoltage(0).withSlot(0);

     

    public EndEffector() {
      PivotEncoder = new CANcoder(5); 

    EndEffectorMotor = new TalonFX(12);
    EndEffectorIntake = new TalonFX(30);

    MotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withFeedback(new FeedbackConfigs()
      .withFeedbackRemoteSensorID(PivotEncoder.getDeviceID()) 
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));

      MotorConfigIntake = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
.withInverted(InvertedValue.Clockwise_Positive));
EndEffectorIntake.getConfigurator().apply(MotorConfigIntake);


      MotorConfig.Slot0.kP = 15;
            MotorConfig.Slot0.kI =1;
       MotorConfig.Slot0.kD = 0;
      // MotorConfig.Slot0 = new Slot0Configs()
      //   .withKP(0.05)
      //   .withKD(0.)
      //   .withKG(0);//THESE ARE FAKE
      //   // .withKA(0.0002)
        // .withKV(0.01)
        // .withKS(0);
    
        EndEffectorMotor.getConfigurator().apply(MotorConfig);


        sensor = new TimeOfFlight(SENSOR_ID);
        sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);


    }

    @Override
  public void periodic() {

    SmartDashboard.putBoolean("EndEffectorSensor", isChoralValid());
    SmartDashboard.putNumber("PivotPosition", getAbsolutePosition());

    SmartDashboard.putNumber("TESTING/elevatorTarget", elevatorTarget);
    SmartDashboard.putNumber("TESTING/elevatorClosedLoopError", Math.abs(elevatorTarget-EndEffectorMotor.getPosition().getValueAsDouble()));
  }

//   public double getElevatorPosition() {

//   }


//   public boolean isElevatorAtPosition(double targetPosition) {
//     return Math.abs(getElevatorPosition() - targetPosition) < ELEVATOR_TOLERANCE;
// }

public void runPercent(DoubleSupplier N_speed) {
        
  double N_speed_D = Math.max( Math.min(N_speed.getAsDouble()*0.3 , 0.25) , -0.25); //0.75
  motorOut.Output = N_speed_D;
  
  EndEffectorMotor.setControl(motorOut);
}

public void setPivotPosition(double targetPosition) {
  if (targetPosition > ElevatorConstants.ELEVATOR_REEF_4_POSITION) {
      targetPosition = ElevatorConstants.ELEVATOR_REEF_4_POSITION;  
  } else if (targetPosition < ElevatorConstants.ELEVATOR_STARTING_POSITION) {
      targetPosition = ElevatorConstants.ELEVATOR_STARTING_POSITION;  
  }
}



    public void setArmPosition(double setpoint) {
      // System.out.println("ARMMMMMMMM");
      EndEffectorMotor.setControl(m_voltagePosition.withPosition(setpoint));
        // setpoint =Math.min( Math.max(setpoint , Constants.Arm.CANAmp ) , Constants.Arm.CANRest);
    //     pid1.setReference(setpoint, ControlType.kPosition);
    //     pid2.setReference(setpoint, ControlType.kPosition);

    }

public double getAbsolutePosition() {
  return EndEffectorMotor.getPosition().getValueAsDouble();
}

  public boolean isPivotAtPosition(double targetPosition) {
    return Math.abs(getAbsolutePosition() - targetPosition) < ELEVATOR_TOLERANCE;
}

public void runIntake(double speed) {
    EndEffectorIntake.set(speed);
}




    public boolean isChoralValid() {
        if (sensor.isRangeValid()) {
            double distance = sensor.getRange();
            return (distance >= PWFdistance);
        } else {
            return false;
        }
    }




    public void stopIntake() {
      EndEffectorIntake.set(0);

    }


}


