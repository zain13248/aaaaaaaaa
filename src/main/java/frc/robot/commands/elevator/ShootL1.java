package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;




public class ShootL1 extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetPosition;
    private final double tolerance = 0.5; // Adjust this based on testing

    public ShootL1(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.targetPosition = Constants.ElevatorConstants.ELEVATOR_REEF_1_POSITION;
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
