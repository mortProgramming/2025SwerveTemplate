package frc.robot.config;

import frc.robot.subsystems.Drivetrain;

import static frc.robot.config.Constants.Drivetrain.*;

import static frc.robot.config.Inputs.*;

import frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IO {

	private static Drivetrain drivetrain;

    public static void init() {
		drivetrain = Drivetrain.getInstance();
        System.out.println("Subsystem init");
    }

    public static void configure() {
        init();
        Inputs.init();

		drivetrain.setDefaultCommand(
			new Drive(Inputs::getJoystickX, Inputs::getJoystickY, Inputs::getJoystickRotate)
        );

       //Drivetrain Field Orient command
        joystick.button(2).whileTrue(new InstantCommand(() -> drivetrain.zeroIMU(), drivetrain));
    }

    public static Boolean getIsBlue() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : true;
	}
 
 }
