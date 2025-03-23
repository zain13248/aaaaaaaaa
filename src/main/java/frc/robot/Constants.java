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

    public static final double Can90_endEffector = 0.0;

    public static final double EndEffector_STARTING_POSITION = 0.02;
    public static final double EndEffector_STARTING_ELEVATOR = 0.2;

    public static final double EndEffector_REEF_1_POSITION = 0.13134765625;
    public static final double EndEffector_REEF_2_POSITION = 0.13134765625;
    
    public static final double EndEffector_REEF_3_POSITION =  0.14990234375
    ;
    public static final double EndEffector_REEF_4_POSITION = 0.13134765625;


    public static final double EndEffector_ALGAE_POSITION = 0.102783203125;
    public static final double EndEffector_ALGAE_2_POSITION = 0.14;
    public static final double EndEffector_ALGAE_3_POSITION = 3;
    public static final double EndEffector_ALGAE_4_POSITION = 4;


    public static final double EndEffector_HUMAN_STATION = 0.05;


    
    public static final double MOTION_MAGIC_ARM_SLOWER_VELOCITY = 320;
    public static final double MOTION_MAGIC_ARM_SLOWER_ACCLERATION = 400;
    public static final double MOTION_MAGIC_ARM_VELOCITY = 320;
    public static final double MOTION_MAGIC_ARM_ACCLERATION = 640;
    public static final double MOTION_MAGIC_ARM_JERK = 0;

    public static final double EndEffector_TOLERANCE = 0.005;

  }



  public final class ElevatorConstants{

    public static final int LEFT_ELEVATOR_MOTOR_1_ID = 25;
    public static final int RIGHT_ELEVATOR_MOTOR_2_ID = 31;
    public static final double HUMAN_PLAYER_STATION_CENTIMETERS = -0.17734375;

    public static final double Can90_Elevator = 0;
    public static final double ELEVATOR_STARTING_POSITION = 0.01;
    public static final double ELEVATOR_REEF_1_POSITION = 2.031982421875;
    public static final double ELEVATOR_REEF_2_POSITION = 6.23505859375;
    public static final double ELEVATOR_REEF_3_POSITION = 10.098388671875;
    public static final double ELEVATOR_REEF_4_POSITION = 0;


    public static final double ELEVATOR_ALGAE_1_POSITION = 3.031982421875;
    public static final double ELEVATOR_ALGAE_2_POSITION = 5.031982421875;
    public static final double ELEVATOR_ALGAE_3_POSITION = 3;
    public static final double ELEVATOR_ALGAE_4_POSITION = 4;

    public static final double ELEVATOR_HUMAN_STATION = 194.5;

    
    public static final double MOTION_MAGIC_ELEVATOR_SLOWER_VELOCITY = 320;
    public static final double MOTION_MAGIC_ELEVATOR_SLOWER_ACCLERATION = 400;
    public static final double MOTION_MAGIC_ELEVATOR_VELOCITY = 320;
    public static final double MOTION_MAGIC_ELEVATOR_ACCLERATION = 640;
    public static final double MOTION_MAGIC_ELEVATOR_JERK = 0;

    public static final double ELEVATOR_TOLERANCE = 0.005;
  }

  public final class AutoAllign{
    public static final double kP = 0.11;
    public static final double kI = 0.0;
    public static double kD = 0.01;

    

  }

}
