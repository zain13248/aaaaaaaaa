package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ArmSubsystem;

public class ShootL4 extends SequentialCommandGroup {

    public ShootL4(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, EndEffector endEffector) {
        addCommands(
            //PUT IN APTIL TAG AUTO ALLIGN HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
            new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_REEF_4_POSITION)),
            new WaitCommand(1.0),

            new InstantCommand(() -> armSubsystem.setArmPosition(Constants.ArmConstants.ARM_REEF_4_POSITION)),
            new WaitCommand(1.0), 
            new InstantCommand(() -> endEffector.runIntake(-1.0)), 
            new WaitCommand(0.5), 
            new InstantCommand(() -> endEffector.stopIntake()),
            

            new InstantCommand(() -> armSubsystem.setArmPosition(Constants.ArmConstants.ARM_STARTING_POSITION)),
            new WaitCommand(1.0),
            new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(Constants.ElevatorConstants.ELEVATOR_STARTING_POSITION))
        );
    }
}
