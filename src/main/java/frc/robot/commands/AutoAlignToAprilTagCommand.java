package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignToAprilTagCommand extends Command {
  /** Creates a new aprilTagSwerve. */
  int targetTag;
  Double tx,ty,ta;
 // Boolean driveMode;
 CommandSwerveDrivetrain swerve;
 limelight limelight;
  double kP,kI,kD;
  PIDController thetaController;
  private long startTime;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  
public AutoAlignToAprilTagCommand(CommandSwerveDrivetrain swerve, limelight limelight)
{

this.swerve = swerve;
this.limelight = limelight;

addRequirements(swerve);
}

  @Override
  public void initialize() {

    startTime = System.currentTimeMillis();


    thetaController = new PIDController(Constants.AutoAllign.kP, Constants.AutoAllign.kI, Constants.AutoAllign.kD);
    getLimelightValues();
  }

  @Override
  public void execute() {
    getLimelightValues();
    printLimelightVal(); 
    double rotation = thetaController.calculate(limelight.getLimelightTX(), 0);

    swerve.applyRequest(() ->
    drive.withVelocityX(0.5 ) 
        .withVelocityY(0 ) 
        .withRotationalRate(rotation)
);
    

  }

  @Override
  public void end(boolean interrupted) {

    swerve.applyRequest(() ->
    drive.withVelocityX(0 ) 
        .withVelocityY(0 ) 
        .withRotationalRate(0)
);
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.getLimelightTX() < 0.005;
    
//    return (System.currentTimeMillis() - startTime) >= (5 * 1000);
  }



  public void getLimelightValues()
  {
    tx= limelight.getLimelightTX();
    ty = limelight.getLimelightTY();
    ta = limelight.getLimelightTA();
  }

  public void printLimelightVal()
  {
    getLimelightValues();
    SmartDashboard.putNumber("Limelight tx", tx);
    SmartDashboard.putNumber("Limelight ty", ty);
    SmartDashboard.putNumber("Limelight ta", ta);
  }
  
}