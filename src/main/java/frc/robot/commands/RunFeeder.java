package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class RunFeeder extends Command {
    private final FeederSubsystem feeder;
    private final double speed;

    public RunFeeder(FeederSubsystem feeder, double speed) {
        this.feeder = feeder;
        this.speed = speed;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.runIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
