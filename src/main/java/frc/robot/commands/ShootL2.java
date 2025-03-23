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

public class ShootL2 extends SequentialCommandGroup {

    public ShootL2(ElevatorSubsystem elevatorSubsystem, EndEffector endEffector) {
        addCommands(
            //PUT IN APTIL TAG AUTO ALLIGN HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
            new ElevatorL2(elevatorSubsystem, endEffector),
            new WaitUntilCommand(() -> (elevatorSubsystem.isElevatorAtPosition(Constants.ElevatorConstants.ELEVATOR_REEF_2_POSITION))),

            new endEffectorL2(endEffector),

            new WaitUntilCommand(() -> (endEffector.isEndEffectorAtPosition(Constants.EndEffector.EndEffector_REEF_2_POSITION)))



        );
    }
}
