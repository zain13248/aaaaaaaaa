package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_REEF_1_POSITION;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.algae.*;
import frc.robot.commands.endEffector.*;
import frc.robot.commands.elevator.*;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;

public class AlgaeL4 extends SequentialCommandGroup {

    public AlgaeL4(ElevatorSubsystem elevatorSubsystem, EndEffector endEffector) {
        addCommands(
            //PUT IN APTIL TAG AUTO ALLIGN HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
            new ElevatorL1(elevatorSubsystem, endEffector),
            new WaitUntilCommand(() -> (elevatorSubsystem.isElevatorAtPosition(Constants.ElevatorConstants.ELEVATOR_ALGAE_1_POSITION))),

            new endEffectorL1(endEffector),
            new WaitUntilCommand(() -> (endEffector.isEndEffectorAtPosition(Constants.EndEffector.EndEffector_REEF_1_POSITION))),

            new InstantCommand(() -> endEffector.runIntake(-1.0)),             
            new WaitCommand(1),
            new InstantCommand(() -> endEffector.stopIntake()),

            
            new ElevatorStarting(elevatorSubsystem),
            new endEffectorStarting(endEffector)

        );
    }
}
