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

public class AlgaeHold extends SequentialCommandGroup {

    public AlgaeHold(EndEffector endEffector) {
        addCommands(
            //PUT IN APTIL TAG AUTO ALLIGN HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
            new InstantCommand(() -> endEffector.runIntake(0.5)),

            new WaitUntilCommand(() -> (endEffector.isCurrentOverThreshold())),

            new InstantCommand(() -> endEffector.runIntake(0.05))


            
        );
    }
}
