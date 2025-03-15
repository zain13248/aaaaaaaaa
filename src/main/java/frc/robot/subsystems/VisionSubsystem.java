package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tv, tx, ty, ta, tid;
    private final String limelightName;
    private Pose2d latestVisionPose;
    private double latestTimestamp;
    private boolean hasValidTarget = false;

    public VisionSubsystem(String limelightName) {
        this.limelightName = limelightName;
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

        // ✅ Get the necessary Limelight values
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tid = limelightTable.getEntry("tid");

    


    }

    public double getTx() {
        return tx.getDouble(0); 
    }
    
    public double getTy() {
        return ty.getDouble(0);
    }


    public boolean hasValidTarget() {
        return hasValidTarget;
    }

    public Pose2d getLatestVisionPose() {
        return latestVisionPose;
    }


       
    public void updateVisionPose() {
        boolean validTarget = tv.getDouble(0) == 1;
        if (!validTarget) {
            hasValidTarget = false;
            return;
        }
    
        Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    
        if (visionPose != null) {
            latestVisionPose = visionPose;
            hasValidTarget = true;
        } else {
            hasValidTarget = false;
        }
    }
    
    
        
    

    @Override
    public void periodic() {


        updateVisionPose();


    }



}

