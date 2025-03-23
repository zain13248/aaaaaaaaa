package frc.robot.commands.endEffector;

import static frc.robot.Constants.EndEffector.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class endEffectorAlgae extends Command {
    private final EndEffector endEffector; 
    private final double targetPosition;
    private final double tolerance = 0.5;

    public endEffectorAlgae(EndEffector endEffector) {
        this.endEffector = endEffector;
        this.targetPosition = Constants.EndEffector.EndEffector_ALGAE_POSITION + Can90_endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(endEffector.getAbsolutePosition() - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
