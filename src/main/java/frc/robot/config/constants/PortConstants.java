// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class PortConstants {
public static final class Controller {
    public static final int JOYSTICK = 0;
    public static final int CONTROLLER = 1;

    public static final int JOYSTICK_X_CHANNEL = 0;
    public static final int JOYSTICK_Y_CHANNEL = 1;
    public static final int JOYSTICK_TWIST_CHANNEL = 2;
    public static final int THROTTLE_CHANNEL = 3;

    public static final double DEAD_BAND = 0;

    public static final double MAX_THROTTLE = 1;
    public static final double MIN_THROTTLE = 0;
    public static final double MAX_ROTATE = 1;
    public static final double MIN_ROTATE = 0;
  }
  
  public static final class Drivetrain {
    public static final int FRONT_LEFT_DRIVE_MOTOR = 0;
    public static final int FRONT_LEFT_STEER_MOTOR = 0;
    public static final int FRONT_LEFT_ENCODER = 0;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 0;
    public static final int FRONT_RIGHT_STEER_MOTOR = 0;
    public static final int FRONT_RIGHT_ENCODER = 0;

    public static final int BACK_LEFT_DRIVE_MOTOR = 0;
    public static final int BACK_LEFT_STEER_MOTOR = 0;
    public static final int BACK_LEFT_ENCODER = 0;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 0;
    public static final int BACK_RIGHT_STEER_MOTOR = 0;
    public static final int BACK_RIGHT_ENCODER = 0;

    public static final int IMU_ID = 0;

    public static final String CANIVORE_NAME = "Drivetrain";

  }
}
