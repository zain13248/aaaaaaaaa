package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_REEF_1_POSITION;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.TunerConstants;
import frc.robot.commands.algae.*;
import frc.robot.commands.endEffector.*;
import frc.robot.commands.elevator.*;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.print;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.limelight;
import frc.robot.commands.*;

import frc.robot.commands.*;  // Removed extra semicolon

public class ActualAutoAllign extends SequentialCommandGroup {
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public ActualAutoAllign(CommandSwerveDrivetrain drivetrain, limelight limelightREEF) {
        addCommands(
            new AutoAllignRotate(drivetrain, limelightREEF),  // Rotate first
            new WaitCommand(0.2),  // Wait for a short time
            new AutoAllignDrive(drivetrain, limelightREEF, 0, 0, 14),  // Start driving
            new WaitCommand(2.01),
            print("test1234"), // Rotate again once alignment is achieved

            new AutoAllignRotate(drivetrain, limelightREEF), 
            print("test123"), // Rotate again once alignment is achieved
            new WaitCommand(3),  // Wait for a short time
            new AutoAllignDrive(drivetrain, limelightREEF, 0, 0, 14)  // Drive again after rotation
        );
    }
}

