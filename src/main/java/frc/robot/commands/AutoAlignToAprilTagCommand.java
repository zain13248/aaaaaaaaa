package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;



public class AutoAlignToAprilTagCommand {

    public static Command createAutoAlignToAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
        return drivetrain.defer((Supplier<Command>) () -> {
            if (!visionSubsystem.hasValidTarget()) {
                System.out.println("No valid AprilTag detected.");
                return Commands.none();
            }

            double targetTx = 0;
            double targetTy = 0;

            double xOffset = ((visionSubsystem.getTx() - targetTx) / targetTx) * 0.5; 
            double yOffset = ((visionSubsystem.getTy() - targetTy) / targetTy) * 0.3;

            Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            double timestamp = Timer.getFPGATimestamp(); 

            drivetrain.addVisionMeasurement(limelightPose, timestamp);
            Pose2d tagPose = drivetrain.getState().Pose;

            Translation2d offset = new Translation2d(xOffset, yOffset).rotateBy(tagPose.getRotation());
            Pose2d currentPose = new Pose2d(tagPose.getTranslation().plus(offset), tagPose.getRotation());

            System.out.println("Detected AprilTag at: " + tagPose);
            System.out.println("Starting pose: " + currentPose);
            System.out.println("Target pose: " + tagPose);

            PathConstraints constraints = new PathConstraints(
                0.35,  
                0.35,  
                Math.toRadians(540), 
                Math.toRadians(720)  
            );

            return AutoBuilder.pathfindToPose(
                tagPose,
                constraints,
                0.0
            );

        }).andThen(AutoBuilder.pathfindToPose(
                drivetrain.getState().Pose, 
                new PathConstraints(0.35, 0.35, Math.toRadians(540), Math.toRadians(720)), 
                0.0
        ));
    }
}


