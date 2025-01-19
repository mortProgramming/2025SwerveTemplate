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
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.GregorianCalendar;
import java.util.Optional;

public class Drivetrain extends SubsystemBase {
    private final SwerveDrive swerveDrivetrain;
    private final NavXSwerve gyro;
    private final SwerveModuleConfiguration[] moduleConfigurations;
    private final SwerveIMU imu;


    public Drivetrain() {

        gyro = new NavXSwerve(NavXComType.kUSB2);

        imu = new SwerveIMU() {

            @Override
            public void clearStickyFaults() {
                gyro.clearStickyFaults();
            }

            @Override
            public void factoryDefault() {
                gyro.factoryDefault();
            }

            @Override
            public Object getIMU() {
                return gyro.getIMU();
            }
        
            @Override
            public MutAngularVelocity getYawAngularVelocity() {
                return gyro.getYawAngularVelocity();
            }

            @Override
            public Optional<Translation3d> getAccel() {
                return gyro.getAccel();
            }

            @Override
            public Rotation3d getRawRotation3d() {
                return gyro.getRawRotation3d();
            }

            @Override
            public Rotation3d getRotation3d() {
                return gyro.getRotation3d();
            }

            @Override
            public void setOffset(Rotation3d rot) {
                gyro.setOffset(rot);
            }

            @Override
            public void setInverted(boolean inverted) {
                gyro.setInverted(inverted);
             

            }
        };

        //FRONT LEFT SWERVE MODULE CONFIG
        TalonFXSwerve frontLeftDriveMotor = new TalonFXSwerve(7,
        "Front Left Drive",
        true, 
        DCMotor.getKrakenX60(1));

        TalonFXSwerve frontLeftAngleMotor = new TalonFXSwerve(8,
        "Front Left Angle",
        false, 
        DCMotor.getKrakenX60(1));

        CANCoderSwerve frontLeftEncoder = new CANCoderSwerve(38);

        TalonFXSwerve frontRightDriveMotor = new TalonFXSwerve(2,
        "Front Right Drive",
        true,
        DCMotor.getKrakenX60(1));



        //FRONT RIGHT SWERVE MODULE CONFIG
        TalonFXSwerve frontRightAngleMotor = new TalonFXSwerve(1,
        "Front Right Angle",
        false, 
        DCMotor.getKrakenX60(1));

        CANCoderSwerve frontRightEncoder = new CANCoderSwerve(35);


        //BACK LEFT SWERVE MODULE CONFIG
        TalonFXSwerve backLeftDriveMotor = new TalonFXSwerve(5,
        "Back Left Drive",
        true, 
        DCMotor.getKrakenX60(1));

        TalonFXSwerve backLeftAngleMotor = new TalonFXSwerve(6,
        "Back Left Angle",
        false, 
        DCMotor.getKrakenX60(1));

        CANCoderSwerve backLeftEncoder = new CANCoderSwerve(12);


        //BACK RIGHT SWERVE MODULE CONFIG

        TalonFXSwerve backRightDriveMotor = new TalonFXSwerve(7, 
        "Back Right Drive",
        true,
        DCMotor.getKrakenX60(1));

        TalonFXSwerve backRightAngleMotor = new TalonFXSwerve(8, 
        "Back Right Angle",
        false,
        DCMotor.getKrakenX60(1));

        CANCoderSwerve backRightEncoder = new CANCoderSwerve(13);


        //OTHER COMPONENTS

        PIDFConfig anglePIDF = new PIDFConfig(0.1, 0.0, 0.0, 0.0);
        PIDFConfig velocityPIDF = new PIDFConfig(1.0, 0.0, 0.0, 0.0);
        ConversionFactorsJson conversionFactors = new ConversionFactorsJson();
        SwerveModulePhysicalCharacteristics physicalCharacteristics = 
        new SwerveModulePhysicalCharacteristics(
            conversionFactors,
            0.7112,
            0.8128);


        SwerveModule frontLeftModule = new SwerveModule(
            1,new SwerveModuleConfiguration(
                frontLeftDriveMotor,
                frontLeftAngleMotor,
                conversionFactors,
                frontLeftEncoder,
                0, 
                0.4064, 
                0.3556, 
                anglePIDF, 
                velocityPIDF, 
                physicalCharacteristics, 
                "Front Left Module", 
                true));

        SwerveModule frontRightModule = new SwerveModule(
            2, new SwerveModuleConfiguration(
                frontRightDriveMotor,
                frontRightAngleMotor,
                conversionFactors,
                frontRightEncoder,
            0,
                0,
                0,
                anglePIDF,
                velocityPIDF,
                physicalCharacteristics,
                "Front Right Module",
                true)
        );

        SwerveModule backLeftModule = new SwerveModule(
            3,new SwerveModuleConfiguration(
                backLeftDriveMotor, 
                backLeftAngleMotor, 
                conversionFactors, 
                backLeftEncoder, 
            0, 
                0, 
                0, 
                anglePIDF, 
                velocityPIDF, 
                physicalCharacteristics, 
                "Back Left Module", 
                true)
        );

        SwerveModule backRightModule = new SwerveModule(
            4,new SwerveModuleConfiguration(
                backRightDriveMotor, 
                backRightAngleMotor, 
                conversionFactors, 
                backRightEncoder, 
            0, 
                0, 
                0, 
                anglePIDF, 
                velocityPIDF, 
                physicalCharacteristics, 
                "Back Right Module", 
                true)
        );


        moduleConfigurations = new SwerveModuleConfiguration[] {
            frontLeftModule.getConfiguration(),
            frontRightModule.getConfiguration(),
            backLeftModule.getConfiguration(),
            backRightModule.getConfiguration()
        };


        SwerveDriveConfiguration drivetrainConfig = new SwerveDriveConfiguration(
            moduleConfigurations, imu, false, physicalCharacteristics
        );

        swerveDrivetrain = new SwerveDrive(drivetrainConfig, 
        null, 
        0, 
        getPose());
    }

    public void drive(Translation2d Speeds, double rot, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrivetrain.drive(Speeds, rot, fieldRelative, isOpenLoop);
    }

    public Pose2d getPose() {
        return swerveDrivetrain.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrivetrain.resetOdometry(pose);
    }

    // public void zeroGyro() {
    //     gyro.set;
    // }

    @Override
    public void periodic() {
        swerveDrivetrain.updateOdometry();
    }
}
