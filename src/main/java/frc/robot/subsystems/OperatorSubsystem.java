package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorSubsystem extends SubsystemBase {
    public enum Position {
        LEVEL_1, LEVEL_2, LEVEL_3, LEVEL_4
    }
    
    public enum Side {
        LEFT, RIGHT
    }

    private Position targetLevel;
    private Side targetSide;

    public OperatorSubsystem() {
        // Default to Level 1 and Left
        targetLevel = Position.LEVEL_1;
        targetSide = Side.LEFT;
    }

    public void setTargetLevel(Position level) {
        this.targetLevel = level;
    }

    public void setTargetSide(Side side) {
        this.targetSide = side;
    }

    public Position getTargetLevel() {
        return targetLevel;
    }

    public Side getTargetSide() {
        return targetSide;
    }
}
