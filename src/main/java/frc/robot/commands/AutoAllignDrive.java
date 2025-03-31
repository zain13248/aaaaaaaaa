package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight;


public class AutoAllignDrive extends Command {
  private limelight m_Limelight;
  private CommandSwerveDrivetrain m_Drivetrain;
  private PIDController xController = new PIDController(0.015, 0.0001, 0.0008);//.0045);
  private PIDController yController = new PIDController(0.035, 0.0001, 0.00004);
  private double targetx;
  private double targety;
  private long startTime;

  private boolean targeting = false;
  private double offset;
  public AutoAllignDrive(CommandSwerveDrivetrain drivetrain, limelight Limelight, double offset, double targetx, double targety) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_Limelight = Limelight;
    this.targetx = targetx;
    this.targety=targety;
    xController.setSetpoint(targetx);
    yController.setSetpoint(targety);
    this.offset = offset;
    SmartDashboard.putBoolean("TARGET", false);
    SmartDashboard.putBoolean("Aligned", false);

  }


private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric() 
  
 // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();

    targeting = false;
    xController.reset();
    xController.setTolerance(0.5);
    yController.reset();
    yController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = 0;
    double ySpeed = 0;
		if (m_Limelight.hasTarget()){
			double z_distance = m_Limelight.getLimelightTA(); //THIS IS IN TERMS OF CAMERA WATCHOUT
			double x_distance = m_Limelight.getLimelightTX();
			 xSpeed = xController.calculate(x_distance, targetx);
       ySpeed = yController.calculate(z_distance, targety);
       SmartDashboard.putNumber("A-limelight", ySpeed);
       SmartDashboard.putNumber("X-limelight", xSpeed);


      //xOutput = -m_throttle.get()*DrivetrainConstants.maxSpeedMetersPerSecond;
		SmartDashboard.putBoolean("TARGET", true);
		}else{
      SmartDashboard.putBoolean("TARGET", false);

    }
    SmartDashboard.putNumber("TX Error", Math.abs(m_Limelight.getLimelightTX() - targetx));
SmartDashboard.putNumber("TA Error", Math.abs(m_Limelight.getLimelightTA() - targety));

    if (Math.abs(m_Limelight.getLimelightTA()-targety) <= 0.4 && Math.abs(m_Limelight.getLimelightTX()-targetx)<=0.25) {
      SmartDashboard.putBoolean("Aligned", true);
    } else {
      SmartDashboard.putBoolean("Aligned", false);
    }
    
    m_Drivetrain.setControl(drive
    .withVelocityX(-ySpeed*(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/3))
    .withVelocityY(xSpeed*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/3)
    .withRotationalRate(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (System.currentTimeMillis() - startTime) >= 2000; // 2 seconds

   
  //  return false; //TODO: FINISH CHECKINHG
   // return Math.abs(m_Limelight.getLimelightTA()-targety) <= 0.4 && Math.abs(m_Limelight.getLimelightTX()-targetx)<=0.25;
  }
}