package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_REEF_1_POSITION;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.endEffector.endEffectorHumanStation;
public class HumanStation extends SequentialCommandGroup {

    public HumanStation(ElevatorSubsystem elevatorSubsystem, EndEffector endEffector, FeederSubsystem feeder) {
        addCommands(
            //PUT IN APTIL TAG AUTO ALLIGN HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
             new endEffectorHumanStation(endEffector),
             new InstantCommand(() -> endEffector.runIntake(-0.5)),
             new InstantCommand(() -> feeder.runIntake(0.3)),

             new WaitUntilCommand(() -> endEffector.isChoralValid()),

             new InstantCommand(() -> endEffector.runMotorAfterChoralValid()),

           //  new InstantCommand(() -> endEffector.runIntake(0)),
             new InstantCommand(() -> feeder.runIntake(0))

        );
    }
}//-2582.8, -2604.097
