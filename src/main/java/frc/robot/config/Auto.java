package frc.robot.config;

// import static frc.robot.config.constants.PIDConstants.Drivetrain.*;
// import static frc.robot.config.constants.PhysicalConstants.Drivetrain.*;

// import frc.robot.commands.autons.odometered.ScoreAmpBlue;
// import frc.robot.commands.autons.odometered.ScoreAmpRed;
// import frc.robot.commands.autons.pathplanned.BasicCommands;
// import frc.robot.commands.autons.timed.blue.TaxiB;
// import frc.robot.commands.autons.timed.red.TaxiR;
// import frc.robot.mortlib.subsystems.swerve.PathPlanner;
import frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {
	private static Drivetrain drivetrain;

	private static SendableChooser<Command> autoChooser;
	
	public static void configure() {
		drivetrain = Drivetrain.getInstance();

		autoChooser = new SendableChooser<Command>();
		configureAutoBuilder();
		addAutoOptions();

		SmartDashboard.putData(autoChooser);
	}

	public static void configureAutoBuilder() {
		// PathPlanner.configure(
		// 	drivetrain, drivetrain.getSwerveDrive(),
		// 	() -> drivetrain.getEstimatedPosition(), (Pose2d startPose) -> Odometer.resetOdometry(startPose),
		// 	new PIDConstants(AUTON_POS_KP, AUTON_POS_KI, AUTON_POS_KD), 
		// 	new PIDConstants(AUTON_ROTATION_KP, AUTON_ROTATION_KI, AUTON_ROTATION_KD), 
		// 	DRIVEBASE_RADIUS_METERS
		// );
	}
	
	public static void addAutoOptions() {
		// By default, the nothing option is selected
		autoChooser.setDefaultOption("nothing", null);

		// autoChooser.addOption("PathPlanner TwoPiece", getPlanned("PathPlanned2Piece"));
	}

	public static Command getPlanned(String plan) {
		// BasicCommands.setCommands();

		return new PathPlannerAuto(plan);
	}

	public static Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
