package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.TunerConstants;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;


import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAllignLeft extends Command {
  int targetTag;
  Double tx, ty, ta;
  CommandSwerveDrivetrain swerve;
  limelight limelight;
  private final XboxController joystick; 

  PIDController thetaController;
  PIDController forwardController;
  PIDController strafeController;

  private long startTime;

  private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();
  private static final double LIMELIGHT_OFFSET = -3;

  public AutoAllignLeft(CommandSwerveDrivetrain swerve, limelight limelight, XboxController joystick) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.joystick = joystick; 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();

    thetaController = new PIDController(0.01, Constants.AutoAllign.kI, Constants.AutoAllign.kD);
    forwardController = new PIDController(0.12, 0.0, 0.0);
    strafeController = new PIDController(0.1, 0.0, 0.0);

    getLimelightValues();
  }

  @Override
  public void execute() {
    getLimelightValues();
    printLimelightVal();

    SmartDashboard.putBoolean("AutoAlign Running", true);
    double rotation = thetaController.calculate(limelight.getLimelightTX(), 0);
    double forwardSpeed = forwardController.calculate(ty, -20);

    forwardSpeed = Math.max(-0.5, Math.min(0.5, forwardSpeed));

    if (ta >= 1.5) {
      forwardSpeed = 0;
    }

    double strafeSpeed = strafeController.calculate(tx, 0);
    strafeSpeed = Math.max(-0.5, Math.min(0.5, strafeSpeed));

    if (Math.abs(tx - LIMELIGHT_OFFSET) < 1.0) {
      strafeSpeed *= 0.3;
    }
    if (Math.abs(tx - LIMELIGHT_OFFSET) < 0.5) {
      strafeSpeed = 0;
    }

    if (Math.abs(tx - LIMELIGHT_OFFSET) < 1.0) {
      rotation *= 0.3;
    }
    if (Math.abs(tx - LIMELIGHT_OFFSET) < 0.5) {
      rotation = 0;
    }

    double manualX = joystick.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double manualY = joystick.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double manualRotation = joystick.getRightX() * RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    swerve.setControl(
        drive.withSpeeds(new ChassisSpeeds(
            manualX,                     
            manualY + strafeSpeed,       
            manualRotation + rotation   
        ))
    );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(
        drive.withSpeeds(new ChassisSpeeds(0, 0, 0))
    );
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= 1000; // 2 seconds
  }

  public void getLimelightValues() {
    tx = limelight.getLimelightTX();
    ty = limelight.getLimelightTY();
    ta = limelight.getLimelightTA();
  }

  public void printLimelightVal() {
    getLimelightValues();
    SmartDashboard.putNumber("Limelight tx", tx);
    SmartDashboard.putNumber("Limelight ty", ty);
    SmartDashboard.putNumber("Limelight ta", ta);
  }
}
