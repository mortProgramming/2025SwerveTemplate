package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.util.Constants.Drivetrain.*;
import static frc.robot.util.Constants.OperatorConstants.*;

import frc.robot.commands.DriveControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
	private final Drivetrain drivetrain;
	// private final Auto auto = Auto.getInstance();
	private final Limelight limelight;

	// private final XboxController xboxController = new
	// XboxController(CONTROLLER_PORT);
	private final CommandJoystick joystick = new CommandJoystick(JOYSTICK_PORT);

	public RobotContainer() {
		drivetrain = Drivetrain.getInstance();
		limelight = Limelight.getInstance();

		drivetrain.setDefaultCommand(
			// new DriveControl(
			// 	() -> (-modifyAxis(joystick.getX(), joystick.getThrottle()) * MAX_VELOCITY_METERS_PER_SECOND),
			// 	() -> (-modifyAxis(joystick.getY(), joystick.getThrottle()) * MAX_VELOCITY_METERS_PER_SECOND),
			// 	() -> (-modifyAxis(joystick.getTwist(), joystick.getThrottle())
			// 			* MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
			// )

			new DriveControl(
				joystick::getX,
				joystick::getY,
				joystick::getTwist
			)
		);

		configureButtonBindings();

		drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0, 0)));

		drivetrain.zeroIMU();
	}

	private void configureButtonBindings() {
		joystick.trigger().whileTrue(new InstantCommand(() -> drivetrain.zeroIMU(), drivetrain));
	}

	public Command getAutonomousCommand() {

		// // Create the AutoBuilder. This only needs to be created once when robot code
		// // starts, not every time you want to create an auto command. A good place to
		// // put this is in RobotContainer along with your subsystems.
		// SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(drivetrain::getPose, // Pose2d supplier
		// 		drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
		// 		drivetrain.driveKinematics, // SwerveDriveKinematics
		// 		new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
		// 											// and Y PID controllers)
		// 		new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
		// 											// rotation controller)
		// 		drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
		// 		eventMap, false, // Should the path be automatically mirrored depending on alliance color.
		// 							// Optional, defaults to true
		// 		drivetrain // The drive subsystem. Used to properly set the requirements of path following
		// 					// commands
		// );

		return null;
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private static double modifyAxis(double value, double throttleValue) {
		// Deadband
		value = deadband(value, 0.1);

		// Square the axis
		value = Math.copySign(value * value, value);

		// takes the throttle value and takes it from [-1, 1] to [0.2, 1], and
		// multiplies it by the
		// value
		return value * (throttleValue * -0.4 + 0.6);
	}
}
