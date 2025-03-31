// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.limelight;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.TunerConstants;
import frc.robot.commands.elevator.ElevatorL1;
import frc.robot.commands.elevator.ElevatorL2;
import frc.robot.commands.elevator.ElevatorL3;
import frc.robot.commands.elevator.ElevatorL0;
import frc.robot.commands.algae.AlgaeFloor;
import frc.robot.commands.algae.AlgaeFloor2;
//import frc.robot.commands.PivotPIDtesting;
import frc.robot.commands.*;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;

//import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
 
 
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    //private final ArmSubsystem armSubsystem = new ArmSubsystem();
    //private final EndEffector endEffector = new EndEffector();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final EndEffector endEffector = new EndEffector();


//Shoot commands
private final ShootL1 shootL1 = new ShootL1(elevatorSubsystem, endEffector);
private final ShootL2 shootL2 = new ShootL2(elevatorSubsystem, endEffector);
private final ShootL3 shootL3 = new ShootL3(elevatorSubsystem, endEffector);
private final ShootL0 shootL0 = new ShootL0(elevatorSubsystem, endEffector);

private final AlgaeL1 algaeL1 = new AlgaeL1(elevatorSubsystem, endEffector);
private final AlgaeL2 algaeL2 = new AlgaeL2(elevatorSubsystem, endEffector);
private final AlgaeL3 algaeL3 = new AlgaeL3(elevatorSubsystem, endEffector);
private final AlgaeL4 algaeL4 = new AlgaeL4(elevatorSubsystem, endEffector);


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


   //   private final SendableChooser<Command> m_chooser = new SendableChooser<>();
   // private final Command MiddleAuto = new PathPlannerAuto("Middle Auto");

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final XboxController joystick = new XboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final limelight limelightREEF = new limelight("limelight-right");
    private final limelight limelightREEF2 = new limelight("limelight-left");



    private final VisionSubsystem visionSubsystem = new VisionSubsystem("limelight_right");
    private final VisionSubsystem visionSubsystemRight = new VisionSubsystem("limelight_left");



    private final SendableChooser<Command> autoChooser;



    public RobotContainer() {

        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Middle Auto", new PathPlannerAuto("Middle Auto"));
  //      autoChooser.addOption("Left Auto", new PathPlannerAuto("Left Auto"));


        SmartDashboard.putData("Auto", autoChooser);







    NamedCommands.registerCommand("runFeeder", new RunFeeder(feederSubsystem, 1));
    NamedCommands.registerCommand("stopFeeder", new RunFeeder(feederSubsystem, -1));
   // NamedCommands.registerCommand("shootL4", new ShootL4(elevatorSubsystem, armSubsystem, endEffector));


   
   configureBindings();

    }

    private void configureBindings() {



// Define a BooleanSupplier for LT being pressed
BooleanSupplier isLtPressed = () -> joystick.getLeftTriggerAxis() > 0.5;


BooleanSupplier isRTPressed = () -> joystick.getRightTriggerAxis() > 0.5;
Trigger rightTrigger = new Trigger(isRTPressed);



new JoystickButton(joystick, 1)
    .onTrue(Commands.either(algaeL1, shootL1, isLtPressed));

new JoystickButton(joystick, 2)
.onTrue(Commands.either(algaeL2, shootL2, isLtPressed));

new JoystickButton(joystick, 4)
    .onTrue(Commands.either(algaeL3, shootL3, isLtPressed));

new JoystickButton(joystick, 3)
    .onTrue(Commands.either(algaeL4, shootL0, isLtPressed));


          //visionSubsystem.setDefaultCommand(new RunCommand(() -> visionSubsystem.updateVisionPose(), visionSubsystem));\

    // comment out other defualtCommand to run the feeder or remove defualtcommands so u can use both
 //       elevatorSubsystem.setDefaultCommand(new RunCommand(() -> elevatorSubsystem.runPercent(joystick.getLeftY()), elevatorSubsystem));
        //  feederSubsystem.setDefaultCommand(new RunCommand(() -> feederSubsystem.runIntake(joystick.getLeftTriggerAxis()), feederSubsystem));
      // endEffector.setDefaultCommand(new RunCommand(() -> endEffector.runPercent(joystick.getRightY()), endEffector));
      //  feederSubsystem.setDefaultCommand(new RunCommand(() -> feederSubsystem.runIntake(joystick.getLeftTriggerAxis()), feederSubsystem));
 //    new JoystickButton(joystick, 9).onTrue(new InstantCommand( ()-> endEffector.runIntake(-1)));
   //  new JoystickButton(joystick, 9).onFalse(new InstantCommand( ()-> endEffector.runIntake(0)));

        

    //   feederSubsystem.setDefaultCommand(new RunCommand(() -> feederSubsystem.runIntake(joystick.getLeftTriggerAxis()), feederSubsystem));


  //  new JoystickButton(joystick, 2).onTrue(new ElevatorL1(elevatorSubsystem));

    // new JoystickButton(joystick, 3).onTrue(new AlgaeFloor(elevatorSubsystem, endEffector));
    // new JoystickButton(joystick, 4).onTrue(new AlgaeFloor2(elevatorSubsystem, endEffector));


   // new JoystickButton(joystick, 2).onTrue(new AlgaeHold(endEffector));

  

    //    new JoystickButton(joystick, 2).onTrue(new InstantCommand( ()-> endEffector.runIntake(0.5)));
    //    new JoystickButton(joystick, 2).onFalse(new InstantCommand( ()-> endEffector.runIntake(0)));

    //    new JoystickButton(joystick, 6).onTrue(new InstantCommand( ()-> endEffector.runIntake(0.05)));
    //    new JoystickButton(joystick, 6).onFalse(new InstantCommand( ()-> endEffector.runIntake(0)));

     //  feederSubsystem.setDefaultCommand(new RunCommand(() -> feederSubsystem.runIntake(joystick.getLeftTriggerAxis()), feederSubsystem));
//new JoystickButton(joystick, 1).onTrue(new (elevatorSubsystem));



//   new JoystickButton(joystick, 3).onTrue(new ElevatorL2(elevatorSubsystem));
//    new JoystickButton(joystick, 1).onTrue(new ElevatorL1(elevatorSubsystem));
//    new JoystickButton(joystick, 4).onTrue(new ElevatorL3(elevatorSubsystem));





        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
 
    //    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()));


        drivetrain.setDefaultCommand(
            // Drivetrai
                drivetrain.applyRequest(() ->

                drive.withVelocityX(joystick.getLeftY() * MaxSpeed ) 
                    .withVelocityY(joystick.getLeftX() * MaxSpeed ) 
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) )
        
        );

        new JoystickButton(joystick, 8).onTrue(new InstantCommand(() -> drivetrain.zeroGyro()
        ));
        
        new JoystickButton(joystick, 6).onTrue(new ActualAutoAllign(drivetrain, limelightREEF));

           new JoystickButton(joystick, 5).whileTrue(new teleopAutoAllign(drivetrain, limelightREEF, joystick));
     //      new JoystickButton(joystick, 5).onTrue(new AutoAllignRotate(drivetrain, limelightREEF));
           new JoystickButton(joystick, 7).onTrue(new ShootL1Out(elevatorSubsystem, endEffector));

           rightTrigger.onTrue(new HumanStation(elevatorSubsystem, endEffector, feederSubsystem));






        

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


          //   new JoystickButton(joystick, 5).onTrue(new InstantCommand( ()->elevatorSubsystem.setPosition(0)));


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
        return autoChooser.getSelected();
    }
        
}

