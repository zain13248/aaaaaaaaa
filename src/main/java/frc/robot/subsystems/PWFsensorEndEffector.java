package frc.robot.subsystems;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PWFsensorEndEffector extends SubsystemBase {
    private final TimeOfFlight sensor;
    private static final int SENSOR_ID = 1; // Set your correct CAN ID here
    private static final double PWFdistance = 200; 
     

    public PWFsensorEndEffector() {
        sensor = new TimeOfFlight(SENSOR_ID);
        sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    public boolean isChoralValid() {
        if (sensor.isRangeValid()) {
            double distance = sensor.getRange();
            return (distance >= PWFdistance);
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("EndEffectorSensor", isChoralValid());

    }
}


