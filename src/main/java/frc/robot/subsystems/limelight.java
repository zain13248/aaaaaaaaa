package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/** Add your docs here. */




    public class limelight extends SubsystemBase {
    private final String limelightName;
    String name;
    NetworkTable table;

    public limelight(String name) {
        this.limelightName = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);    

    }





    public double getLimelightTX(){
        return this.table.getEntry("tx").getDouble(0);
    }
    public void setThrottle(int value) {
        this.table.getEntry("throttle_set").setNumber(value);
    }
    public double getThrottle() {
        return this.table.getEntry("throttle_set").getDouble(-1);
    }
    
    public double getLimelightTY(){
        return this.table.getEntry("ty").getDouble(0);
    }
    public double getLimelightTA(){
        return this.table.getEntry("ta").getDouble(0);
    }
    public void lightsOff(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3); //1
    }
    public void lightsOn(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3); //0
    }
    public void lightsForceOn(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3);
    }

        public void lightsForceOnBlink(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3); //2
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }
    
    /** Checks if Limelight has a valid target */


    /** Gets the horizontal offset from the crosshair to the target in degrees */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    /** Gets the vertical offset from the crosshair to the target in degrees */
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    /** Gets the AprilTag ID detected */
    public double getFiducialID() {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    /** Gets the robot's estimated pose from Limelight */
    public Pose2d getBotPose2D() {
        return LimelightHelpers.getBotPose2d(limelightName);
    }

    /** Gets the robot's estimated 3D pose */
    public Pose3d getBotPose3D() {
        return LimelightHelpers.getBotPose3d(limelightName);
    }

    /** Gets the target's 3D pose relative to the camera */
    public Pose3d getTargetPoseCameraSpace() {
        return LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
    }

    /** Gets the target's 3D pose relative to the robot */
    public Pose3d getTargetPoseRobotSpace() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
    }

    /** Gets the X position of the target relative to the robot */
    public double getXOffset() {
        return getTargetPoseRobotSpace().getX();
    }

    /** Gets the Y position of the target relative to the robot */
    public double getYOffset() {
        return getTargetPoseRobotSpace().getY();
    }

    /** Gets the robot's current rotation from Limelight */
    public Rotation2d getRotation() {
        return getBotPose2D().getRotation();
    }
}


    







