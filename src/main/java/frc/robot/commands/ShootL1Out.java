

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

public class ShootL1Out extends SequentialCommandGroup {

    public ShootL1Out(ElevatorSubsystem elevatorSubsystem, EndEffector endEffector) {
        addCommands(


       new InstantCommand(() -> endEffector.runIntake(-1)),             
       new WaitCommand(1),

        new endEffectorStarting(endEffector),
        new WaitUntilCommand(() -> (endEffector.isEndEffectorAtPosition(Constants.EndEffector.EndEffector_STARTING_POSITION))),

        new ElevatorStarting(elevatorSubsystem),
        new InstantCommand(() -> endEffector.runIntake(0))
        


        );
    }
}