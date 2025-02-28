package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignToAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private PathPlannerTrajectory trajectory;
    private boolean isTrajectoryGenerated = false;

    public AutoAlignToAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        addRequirements(drivetrain, visionSubsystem);
    }

    @Override
    public void initialize() {
        if (!visionSubsystem.hasValidTarget()) {
            System.out.println("no april tag");
            cancel();
            return;
        }
        double targetTx = 0;  
        double targetTy = -0;
        
        double xOffset = ((visionSubsystem.getTx() - targetTx) / targetTx) * 0.5; 
        double yOffset = ((visionSubsystem.getTy() - targetTy) / targetTy) * 0.3;
        

        Pose2d tagPose = visionSubsystem.getLatestVisionPose();

        Translation2d offset = new Translation2d(xOffset, yOffset).rotateBy(tagPose.getRotation());

        Pose2d currentPose = new Pose2d(tagPose.getTranslation().plus(offset), tagPose.getRotation()); 

        System.out.println("Detected aprilTag at: " + tagPose);
        System.out.println("starting pose: " + currentPose);
        System.out.println("Target pose: " + tagPose);


  


        new Thread(() -> {
            generateTrajectory(currentPose, tagPose);
        }).start();
    
    }
    

    private void generateTrajectory(Pose2d currentPose, Pose2d tagPose) {


        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            tagPose
        );

        PathConstraints constraints = new PathConstraints(
            0.35,  
            0.35, 
            Math.toRadians(540), 
            Math.toRadians(720)  
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, 
            new GoalEndState(0.0, tagPose.getRotation()) 
        );

        path.preventFlipping = false;

        this.trajectory = new PathPlannerTrajectory(
            path,
            drivetrain.getCurrentChassisSpeeds(),
            currentPose.getRotation(),
            drivetrain.getRobotConfig()
        );
        drivetrain.followTrajectory(this.trajectory); 
        

        isTrajectoryGenerated = true;
        System.out.println("Path is generated");

    }

    @Override
    public void execute() {
        if (!isTrajectoryGenerated || trajectory == null) { 
            System.out.println("Waiting for path");
            return;
        }
        


    }

    @Override
    public boolean isFinished() {
         double maxTx = visionSubsystem.getTx();
         double maxTy = visionSubsystem.getTy();
        
        // if (Math.abs(visionSubsystem.getTx()) >= maxTx * 0.95 || 
        //     Math.abs(visionSubsystem.getTy()) >= maxTy * 0.95) {
        //     return true;
        // }

        if (Math.abs(visionSubsystem.getTx() - maxTx) < 0.5 && 
        Math.abs(visionSubsystem.getTy() - maxTy) < 0.5) { 
        System.out.println("DONE");
        return true;
    }
       
    
        return false; 
    }
    

    @Override
    public void end(boolean interrupted) {




    }
}
