package frc.robot.config;

// import frc.robot.commands.autons.pathPlanner.GetPlanned;
import frc.robot.commands.autons.timed.Taxi;
import frc.robot.MORTlib.subsystems.swerve.PathPlanner;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.config.constants.PIDConstants.Drivetrain.*;
import static frc.robot.config.constants.PhysicalConstants.Drivetrain.*;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
// https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
// https://software-metadata.revrobotics.com/REVLib-2025.json

public class Auto {

	private static Drivetrain drivetrain;

	private static SendableChooser<Command> autoChooser;
	
	public static void configure() {
		drivetrain = Drivetrain.getInstance();

		configureAutoBuilder();
		addAutoOptions();
	}

	public static void configureAutoBuilder() {
		drivetrain.setGyroscopeZero(0);

		PathPlanner.configure(
			drivetrain, drivetrain.getSwerveDrive(),
			new PIDConstants(AUTON_POS_KP, AUTON_POS_KI, AUTON_POS_KD), 
			new PIDConstants(AUTON_ROTATION_KP, AUTON_ROTATION_KI, AUTON_ROTATION_KD),
			WHEEL_COEFFICIENT_OF_FRICTION, DRIVE_MOTOR_CURRENT_LIMIT,
			ROBOT_MASS, ROBOT_MOMENT_OF_INERTIA
		);
	}
	
	public static void addAutoOptions () {
		autoChooser = new SendableChooser<Command>();

		autoChooser.setDefaultOption("nothing", null);

		autoChooser.addOption("Forward", new Taxi());
		// autoChooser.addOption("Circle", GetPlanned.getCircle());

		SmartDashboard.putData(autoChooser);
	}

	public static Command getAutonomousCommand () {
		return autoChooser.getSelected();
	}
}
