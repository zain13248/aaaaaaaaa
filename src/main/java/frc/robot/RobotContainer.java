// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.limelight;

import frc.robot.TunerConstants;
import frc.robot.commands.AutoAlignToAprilTagCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.ShootL4;

import frc.robot.commands.AutoAlignToAprilTagCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.FeederSubsystem;

import frc.robot.commands.PivotPIDtesting;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final XboxController joystick = new XboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final limelight limelightREEF = new limelight("limelight");


    private final VisionSubsystem visionSubsystem = new VisionSubsystem("limelight");
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final EndEffector endEffector = new EndEffector();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();


    private final AutoAlignToAprilTagCommand alignCommand = new AutoAlignToAprilTagCommand();

    public RobotContainer() {
        configureBindings();

    NamedCommands.registerCommand("runFeeder", new RunFeeder(feederSubsystem, 1));
    NamedCommands.registerCommand("stopFeeder", new RunFeeder(feederSubsystem, -1));
    NamedCommands.registerCommand("shootL4", new ShootL4(elevatorSubsystem, armSubsystem, endEffector));


    }

    private void configureBindings() {

            visionSubsystem.setDefaultCommand(new RunCommand(() -> visionSubsystem.updateVisionPose(), visionSubsystem));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

    //    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()));


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed ) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed ) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );





        

        // endEffector.setDefaultCommand(endEffector.run(() ->endEffector.runPercent(() -> -joystick.getLeftY())));


        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //     // reset the field-centric heading on left bumper press
    //     //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    //   new JoystickButton(joystick, 6).onTrue( //rt
    //     new SequentialCommandGroup(
    //         new InstantCommand( ()->endEffector.setArmPosition(0.17)) , //end out
    //         new WaitCommand(1),
    //         new InstantCommand( ()->armSubsystem.setArmPosition(0.2)), // arm up
    //         new WaitCommand(1),
    //         new InstantCommand( ()->elevatorSubsystem.setElevatorPosition(-0.42)), //0
    //         new WaitCommand(1))
    //         );


    //         new JoystickButton(joystick, 5).onTrue(// Lt
    //             new SequentialCommandGroup(
    //                 new InstantCommand( ()->endEffector.setArmPosition(0.17)) , //end out
    //                 new WaitCommand(1),
    //                 new InstantCommand( ()->armSubsystem.setArmPosition(0.2)), // arm up
    //                 new WaitCommand(1),
    //                 new InstantCommand( ()->elevatorSubsystem.setElevatorPosition(2.2)), // human station
    //                 new WaitCommand(1),
    //                 new InstantCommand( ()->endEffector.setArmPosition(0.3)) , // end feeding
    //                 new WaitCommand(1))


    //                 );

    //                 new JoystickButton(joystick, 1).onTrue( // a
    //                     new SequentialCommandGroup(
    //                         new InstantCommand( ()->endEffector.setArmPosition(0.2)) , // end mid
    //                         new WaitCommand(1),
    //                         new InstantCommand( ()->armSubsystem.setArmPosition(0.2)), // arm up
    //                         new WaitCommand(1),
    //                         new InstantCommand( ()->elevatorSubsystem.setElevatorPosition(2)), //l2
    //                         new WaitCommand(1),
    //                         new InstantCommand( ()->endEffector.setArmPosition(0.3)) , // end effector
    //                         new WaitCommand(1))
        
        
    //                         );
    //                         new JoystickButton(joystick, 8).onTrue( 
    //                         new SequentialCommandGroup(
    //                             new InstantCommand( ()->endEffector.setArmPosition(0.2)) , // end mid
    //                             new WaitCommand(1),
    //                             new InstantCommand( ()->armSubsystem.setArmPosition(0.2)), // arm up
    //                             new WaitCommand(1),
    //                             new InstantCommand( ()->elevatorSubsystem.setElevatorPosition(4
    //                             )), //l2
    //                             new WaitCommand(1),
    //                             new InstantCommand( ()->endEffector.setArmPosition(0.3)) , // end effector
    //                             new WaitCommand(1))
            
            
    //                             );
    //   new JoystickButton(joystick, 5).whileTrue(new InstantCommand(() ->armSubsystem.setArmPosition(0.11)));
    //    new JoystickButton(joystick, 2).onTrue(new InstantCommand(() -> endEffector.runIntake(0.5))); // outake B
    //    new JoystickButton(joystick, 2).onFalse(new InstantCommand(() -> endEffector.runIntake(0)));

    //    new JoystickButton(joystick, 7).onTrue(new InstantCommand(() -> endEffector.runIntake(-0.5))); // intake menu
    //    new JoystickButton(joystick, 7).onFalse(new InstantCommand(() -> endEffector.runIntake(0)));

       new JoystickButton(joystick, 1).onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));

    //    new JoystickButton(joystick, 3).onTrue(// Lt
    //    new SequentialCommandGroup(
    //        new InstantCommand( ()->endEffector.setArmPosition(0.35)) , //end out
    //        new WaitCommand(1),
    //        new InstantCommand( ()->armSubsystem.setArmPosition(0.2)), // arm up
    //        new WaitCommand(1),
    //        new InstantCommand( ()->elevatorSubsystem.setElevatorPosition(0)), // human station
    //        new WaitCommand(1))


    //        );
    //    new JoystickButton(joystick, 3).onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(-0.42333984375))); // zero elevator x 
    //    new JoystickButton(joystick, 4).onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(1.25))); // random

    //   new JoystickButton(joystick, 2).whileTrue(new PivotPIDtesting(elevatorSubsystem, armSubsystem, endEffector));

        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return drivetrain.applyRequest(() ->
        drive.withVelocityX(-1) 
            .withVelocityY(0) 
            




            
            .withRotationalRate(0)); 
    

    
    }

    // public Command getAutonomousCommand() {
    //     return new SequentialCommandGroup(
    //         // Move forward with the correct drive directions
    //         new InstantCommand(() -> drivetrain.setControl(( new SwerveRequest.(1.0, 1.0, -1.0, -1.0))),    
    //         new WaitCommand(2.5), // Move for 2.5 seconds
    
    //         // Stop movement
    //         new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.ModuleRequest()
    //             .withModuleSpeeds(0.0, 0.0, 0.0, 0.0)))
    //     );
    // }
}
