package frc.robot.subsystems;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class FeederSubsystem extends SubsystemBase {
    private final TalonFX FeederMotor;
    private TalonFXConfiguration MotorConfig;

    public FeederSubsystem() {
        FeederMotor = new TalonFX(28); //22

    MotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive));

        FeederMotor.getConfigurator().apply(MotorConfig);

    }
    public void runIntake(double speed) {
        FeederMotor.set(speed);
    }

    public void stopIntake() {
        FeederMotor.set(0);

    }


}



