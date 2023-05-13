// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

  public static final class OperatorInputConstants {
  
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
  }

  public static final class DriveConstants {
      
    public static final int LEFT_MOTOR = 3;
    public static final int LEFT_MOTOR_FOLLOWER = 4;
    public static final int RIGHT_MOTOR = 1;
    public static final int RIGHT_MOTOR_FOLLOWER = 2;

    public static final boolean LEFT_MOTOR_REVERSED = false;
    public static final boolean RIGHT_MOTOR_REVERSED = true;

    public static final double  CM_PER_ENCODER_COUNT = 3.503;   // change value after testing
  }

  public static final class ArmConstants {

    public static final int ARM_MOTOR = 6;
    public static final int ARM_MOTOR_FOLLOWER = 5;
    public static final int ARM_LIMIT_SWITCH_FRONT = 0;
    public static final int ARM_LIMIT_SWITCH_BACK = 2;
  }

  public static final class WristConstants {

    public static final int WRIST_MOTOR = 7;
    public static final int WRIST_LIMIT_SWITCH = 6;
  }

}
