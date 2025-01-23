package frc.robot.util;

import edu.wpi.first.math.util.Units;

public final class Constants {
	public static final class OperatorConstants {
		public static final int JOYSTICK_PORT = 0;
		public static final int CONTROLLER_PORT = 2;

		public static final int RESET_GYRO_BUTTON = 1; // on joystick 0
	}
	public static final class Drivetrain {
		// Front Left
		public static final int FRONT_LEFT_DRIVE_MOTOR = 7;
		public static final int FRONT_LEFT_STEER_MOTOR = 8;
		public static final int FRONT_LEFT_ENCODER = 38;
		public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(0);
		// Front Right
		public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
		public static final int FRONT_RIGHT_STEER_MOTOR = 1;
		public static final int FRONT_RIGHT_ENCODER = 35;
		public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(0);
		// Back Left
		public static final int BACK_LEFT_DRIVE_MOTOR = 5;
		public static final int BACK_LEFT_STEER_MOTOR = 6;
		public static final int BACK_LEFT_ENCODER = 37;
		public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(0);
		// Back Right
		public static final int BACK_RIGHT_DRIVE_MOTOR = 3;
		public static final int BACK_RIGHT_STEER_MOTOR = 4;
		public static final int BACK_RIGHT_ENCODER = 36;
		public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(0);

		public static final String DRIVETRAIN_CANBUS = "drivetrain";

		// The left-to-right distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.75);
		// The front-to-back distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.75);

		public static final double MAX_VOLTAGE = 12.0;
		public static final double SECONDS_PER_MINUTE = 60;

		public static final double MAX_DRIVE_MOTOR_RPM = 6000;
		public static final double DRIVE_REDUCTION = (14/50) * (28 / 16) * (15 / 45);
		public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

		public static final double MAX_VELOCITY_METERS_PER_SECOND = MAX_DRIVE_MOTOR_RPM / SECONDS_PER_MINUTE
				* DRIVE_REDUCTION
				* WHEEL_DIAMETER * Math.PI;
		public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
				/ Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);


		public static final double VELOCITY_PIDF_P = 1;
		public static final double VELOCITY_PIDF_I = 0;
		public static final double VELOCITY_PIDF_D = 0;
		public static final double VELOCITY_PIDF_F = 0;
		public static final double DRIVE_MOTOR_RAMP_RATE = 0.7112;

		public static final double ANGLE_PIDF_P = 0.1;
		public static final double ANGLE_PIDF_I = 0;
		public static final double ANGLE_PIDF_D = 0;
		public static final double ANGLE_PIDF_F = 0;
		public static final double STEER_MOTOR_RAMP_RATE = 0.8128;

		public static final double HEADING_PIDF_P = 0.1;
		public static final double HEADING_PIDF_I = 0;
		public static final double HEADING_PIDF_D = 0;
		public static final double HEADING_PIDF_F = 0;

		public static final boolean INVERT_IMU = false;
	}

	public static final class VisionConstants {
		public static final double LIMELIGHT_PITCH = Units.degreesToRadians(15);
		public static final double NODE_APRILTAG_HEIGHT = Units.inchesToMeters(27.5);
		public static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(15.2);

		public static final double DISTANCE_AWAY = Units.inchesToMeters(43);

	}

	public enum LimelightPipeline {
		APRILTAG(1), REFLECTIVE(2), DRIVER(3);

		private final int id;

		LimelightPipeline(int id) {
			this.id = id;
		}

		public int id() {
			return id;
		}
	}
}
