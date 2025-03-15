// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public final class EndEffector{

    public static final int EndEffector_MOTOR_1_ID = 12;

    public static final double EndEffector_STARTING_POSITION = 80;
    public static final double EndEffector_REEF_1_POSITION = -0.65;
    public static final double EndEffector_REEF_2_POSITION = -0.86;
    public static final double EndEffector_REEF_3_POSITION = 187;
    public static final double EndEffector_REEF_4_POSITION = 194.5;

    public static final double EndEffector_HUMAN_STATION = 0.560546875;


    
    public static final double MOTION_MAGIC_ARM_SLOWER_VELOCITY = 320;
    public static final double MOTION_MAGIC_ARM_SLOWER_ACCLERATION = 400;
    public static final double MOTION_MAGIC_ARM_VELOCITY = 320;
    public static final double MOTION_MAGIC_ARM_ACCLERATION = 640;
    public static final double MOTION_MAGIC_ARM_JERK = 0;

    public static final double EndEffector_TOLERANCE = 0.005;
  }

  public final class ArmConstants{


    public static final int CANcoder = 7;

    public static final int ELEVATOR_MOTOR_1_ID = 13;
    public static final double ARM_STARTING_POSITION = 80;
    public static final double ARM_REEF_1_POSITION = -0.133;
    public static final double ARM_REEF_2_POSITION = 0.139;


    public static final double ARM_REEF_3_POSITION = 187;
    public static final double ARM_REEF_4_POSITION = 194.5;

    public static final double ARM_HUMAN_STATION = 0.1462402;

    
    public static final double MOTION_MAGIC_ARM_SLOWER_VELOCITY = 320;
    public static final double MOTION_MAGIC_ARM_SLOWER_ACCLERATION = 400;
    public static final double MOTION_MAGIC_ARM_VELOCITY = 320;
    public static final double MOTION_MAGIC_ARM_ACCLERATION = 640;
    public static final double MOTION_MAGIC_ARM_JERK = 0;

    public static final double ARM_TOLERANCE = 0.005;
  }

  public final class ElevatorConstants{
    public static final int ELEVATOR_MOTOR_1_ID = 25;
    public static final int ELEVATOR_MOTOR_2_ID = 10;
    public static final double HUMAN_PLAYER_STATION_CENTIMETERS = -0.17734375;

    
    public static final double ELEVATOR_STARTING_POSITION = 5.0;
    public static final double ELEVATOR_REEF_1_POSITION = 81.23;
    public static final double ELEVATOR_REEF_2_POSITION = 122.6;
    public static final double ELEVATOR_REEF_3_POSITION = 187;
    public static final double ELEVATOR_REEF_4_POSITION = 194.5;

    public static final double ELEVATOR_HUMAN_STATION = 194.5;

    
    public static final double MOTION_MAGIC_ELEVATOR_SLOWER_VELOCITY = 320;
    public static final double MOTION_MAGIC_ELEVATOR_SLOWER_ACCLERATION = 400;
    public static final double MOTION_MAGIC_ELEVATOR_VELOCITY = 320;
    public static final double MOTION_MAGIC_ELEVATOR_ACCLERATION = 640;
    public static final double MOTION_MAGIC_ELEVATOR_JERK = 0;

    public static final double ELEVATOR_TOLERANCE = 0.005;
  }

  public final class AutoAllign{
    public static final double kP = .012;
    public static final double kI = 0.0;
    public static double kD = 0;

    

  }

}
