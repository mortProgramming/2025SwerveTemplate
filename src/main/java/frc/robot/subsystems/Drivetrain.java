package frc.robot.subsystems;

import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.TalonFXSwerve;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.NavXSwerve;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.json.modules.ConversionFactorsJson;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveDriveConfiguration;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.util.Constants.Drivetrain.*;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;

    private SwerveDrive swerveDrive;
    private NavXSwerve imu;
    private SwerveModuleConfiguration[] moduleConfigurations;
    // private final SwerveIMU imu;


    public Drivetrain() {
        imu = new NavXSwerve(NavXComType.kMXP_SPI);

        // imu = new SwerveIMU() {

        //     @Override
        //     public void clearStickyFaults() {
        //         imu.clearStickyFaults();
        //     }

        //     @Override
        //     public void factoryDefault() {
        //         imu.factoryDefault();
        //     }

        //     @Override
        //     public Object getIMU() {
        //         return imu.getIMU();
        //     }
        
        //     @Override
        //     public MutAngularVelocity getYawAngularVelocity() {
        //         return imu.getYawAngularVelocity();
        //     }

        //     @Override
        //     public Optional<Translation3d> getAccel() {
        //         return imu.getAccel();
        //     }

        //     @Override
        //     public Rotation3d getRawRotation3d() {
        //         return imu.getRawRotation3d();
        //     }

        //     @Override
        //     public Rotation3d getRotation3d() {
        //         return imu.getRotation3d();
        //     }

        //     @Override
        //     public void setOffset(Rotation3d rot) {
        //         imu.setOffset(rot);
        //     }

        //     @Override
        //     public void setInverted(boolean inverted) {
        //         imu.setInverted(inverted);
        //     }
        // };

        //FRONT LEFT SWERVE MODULE CONFIG
        TalonFXSwerve frontLeftDriveMotor = new TalonFXSwerve(FRONT_LEFT_DRIVE_MOTOR,
        DRIVETRAIN_CANBUS,
        true, 
        DCMotor.getKrakenX60(1));

        TalonFXSwerve frontLeftSteerMotor = new TalonFXSwerve(FRONT_LEFT_STEER_MOTOR,
        DRIVETRAIN_CANBUS,
        false, 
        DCMotor.getKrakenX60(1));

        CANCoderSwerve frontLeftEncoder = new CANCoderSwerve(FRONT_LEFT_ENCODER, DRIVETRAIN_CANBUS);




        //FRONT RIGHT SWERVE MODULE CONFIG
        TalonFXSwerve frontRightDriveMotor = new TalonFXSwerve(FRONT_RIGHT_DRIVE_MOTOR,
        DRIVETRAIN_CANBUS,
        true,
        DCMotor.getKrakenX60(1));

        TalonFXSwerve frontRightSteerMotor = new TalonFXSwerve(FRONT_RIGHT_STEER_MOTOR,
        DRIVETRAIN_CANBUS,
        false, 
        DCMotor.getKrakenX60(1));

        CANCoderSwerve frontRightEncoder = new CANCoderSwerve(FRONT_RIGHT_ENCODER, DRIVETRAIN_CANBUS);


        //BACK LEFT SWERVE MODULE CONFIG
        TalonFXSwerve backLeftDriveMotor = new TalonFXSwerve(BACK_LEFT_DRIVE_MOTOR,
        DRIVETRAIN_CANBUS,
        true, 
        DCMotor.getKrakenX60(1));

        TalonFXSwerve backLeftSteerMotor = new TalonFXSwerve(BACK_LEFT_STEER_MOTOR,
        DRIVETRAIN_CANBUS,
        false, 
        DCMotor.getKrakenX60(1));

        CANCoderSwerve backLeftEncoder = new CANCoderSwerve(BACK_LEFT_ENCODER, DRIVETRAIN_CANBUS);


        //BACK RIGHT SWERVE MODULE CONFIG

        TalonFXSwerve backRightDriveMotor = new TalonFXSwerve(BACK_RIGHT_DRIVE_MOTOR, 
        DRIVETRAIN_CANBUS,
        true,
        DCMotor.getKrakenX60(1));

        TalonFXSwerve backRightSteerMotor = new TalonFXSwerve(BACK_RIGHT_STEER_MOTOR, 
        DRIVETRAIN_CANBUS,
        false,
        DCMotor.getKrakenX60(1));

        CANCoderSwerve backRightEncoder = new CANCoderSwerve(BACK_RIGHT_ENCODER, DRIVETRAIN_CANBUS);

        //OTHER COMPONENTS
        PIDFConfig anglePIDF = new PIDFConfig(ANGLE_PIDF_P, ANGLE_PIDF_I, ANGLE_PIDF_D, ANGLE_PIDF_F);
        PIDFConfig velocityPIDF = new PIDFConfig(VELOCITY_PIDF_P, VELOCITY_PIDF_I, VELOCITY_PIDF_D, VELOCITY_PIDF_F);
        PIDFConfig headingPIDF = new PIDFConfig(HEADING_PIDF_P, HEADING_PIDF_I, HEADING_PIDF_D, HEADING_PIDF_F);
        ConversionFactorsJson conversionFactors = new ConversionFactorsJson();
        SwerveModulePhysicalCharacteristics physicalCharacteristics = 
        new SwerveModulePhysicalCharacteristics(
            conversionFactors,
            DRIVE_MOTOR_RAMP_RATE,
            STEER_MOTOR_RAMP_RATE
        );


        SwerveModule frontLeftModule = new SwerveModule(
            1,new SwerveModuleConfiguration(
                frontLeftDriveMotor,
                frontLeftSteerMotor,
                conversionFactors,
                frontLeftEncoder,
                FRONT_LEFT_STEER_OFFSET, 
                DRIVETRAIN_WHEELBASE_METERS / 2, 
                DRIVETRAIN_TRACKWIDTH_METERS / 2, 
                anglePIDF, 
                velocityPIDF, 
                physicalCharacteristics, 
                "Front Left Module", 
                true
            )
        );

        SwerveModule frontRightModule = new SwerveModule(
            2, new SwerveModuleConfiguration(
                frontRightDriveMotor,
                frontRightSteerMotor,
                conversionFactors,
                frontRightEncoder,
                FRONT_RIGHT_STEER_OFFSET,
                DRIVETRAIN_WHEELBASE_METERS / 2, 
                DRIVETRAIN_TRACKWIDTH_METERS / 2, 
                anglePIDF,
                velocityPIDF,
                physicalCharacteristics,
                "Front Right Module",
                true
            )
        );

        SwerveModule backLeftModule = new SwerveModule(
            3,new SwerveModuleConfiguration(
                backLeftDriveMotor, 
                backLeftSteerMotor, 
                conversionFactors, 
                backLeftEncoder, 
                BACK_LEFT_STEER_OFFSET, 
                DRIVETRAIN_WHEELBASE_METERS / 2, 
                DRIVETRAIN_TRACKWIDTH_METERS / 2, 
                anglePIDF, 
                velocityPIDF, 
                physicalCharacteristics, 
                "Back Left Module", 
                true
            )
        );

        SwerveModule backRightModule = new SwerveModule(
            4,new SwerveModuleConfiguration(
                backRightDriveMotor, 
                backRightSteerMotor, 
                conversionFactors, 
                backRightEncoder, 
                BACK_RIGHT_STEER_OFFSET, 
                DRIVETRAIN_WHEELBASE_METERS / 2, 
                DRIVETRAIN_TRACKWIDTH_METERS / 2, 
                anglePIDF, 
                velocityPIDF, 
                physicalCharacteristics, 
                "Back Right Module", 
                true
            )
        );


        moduleConfigurations = new SwerveModuleConfiguration[] {
            frontLeftModule.getConfiguration(),
            frontRightModule.getConfiguration(),
            backLeftModule.getConfiguration(),
            backRightModule.getConfiguration()
        };


        SwerveDriveConfiguration drivetrainConfig = new SwerveDriveConfiguration(
            moduleConfigurations, imu, INVERT_IMU, physicalCharacteristics
        );

        SwerveControllerConfiguration swerveControllerConfiguration = 
            new SwerveControllerConfiguration(
                drivetrainConfig, headingPIDF, MAX_VELOCITY_METERS_PER_SECOND
            );

        swerveDrive = new SwerveDrive(drivetrainConfig, 
        swerveControllerConfiguration, 
        MAX_VELOCITY_METERS_PER_SECOND, 
        new Pose2d(0, 0, new Rotation2d()));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(
            new Translation2d(
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond
            ), 
            chassisSpeeds.omegaRadiansPerSecond, true, true
        );
    }

    public void driveRobotOriented(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(
            new Translation2d(
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond
            ), 
            chassisSpeeds.omegaRadiansPerSecond, false, false
        );
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public void zeroIMU() {
        swerveDrive.zeroGyro();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }

    public static Drivetrain getInstance() {
		if (drivetrain == null) {
			drivetrain = new Drivetrain();
		}
		return drivetrain;
	}
}
