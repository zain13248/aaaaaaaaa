package frc.robot.commands.elevator;

import static frc.robot.Constants.ElevatorConstants.Can90_Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorStarting extends Command {
    private final ElevatorSubsystem elevator; 
    private final double targetPosition;
    private final double tolerance = 0.5;

    public ElevatorStarting(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.targetPosition = Constants.ElevatorConstants.ELEVATOR_STARTING_POSITION + Can90_Elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getAbsolutePosition() - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
