package frc.robot.commands.elevator;

import static frc.robot.Constants.ElevatorConstants.Can90_Elevator;

import static frc.robot.Constants.EndEffector.Can90_endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorL0 extends Command {
    private final ElevatorSubsystem elevator; 
    private final EndEffector endEffector; 

    private final double targetPositionElevator;
    private final double targetPositionEndEffector;

    private final double tolerance = 0.5;

    public ElevatorL0(ElevatorSubsystem elevator, EndEffector endEffector) {
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.targetPositionElevator = Constants.ElevatorConstants.ELEVATOR_REEF_4_POSITION + Can90_Elevator;
        this.targetPositionEndEffector = Constants.EndEffector.EndEffector_REEF_4_POSITION + Can90_endEffector;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
       elevator.setElevatorPosition(targetPositionElevator);
     endEffector.setEndEffectorPosition(targetPositionEndEffector);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getAbsolutePosition() - targetPositionEndEffector) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
