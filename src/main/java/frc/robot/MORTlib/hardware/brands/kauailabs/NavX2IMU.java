package frc.robot.MORTlib.hardware.brands.kauailabs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.MORTlib.hardware.imu.IMUIntf;

import com.studica.frc.AHRS;

public class NavX2IMU implements IMUIntf {

    public AHRS imu;

    public int ID;

    public NavX2IMU(int ID) {
        this.ID = ID;

        imu = new AHRS(AHRS.NavXComType.kMXP_SPI);
    }

    public void setCanivore (String canivore) {
        System.out.println("Why are you here?");
    }

    public double getAngle() {
		if (imu.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return 360.0 - imu.getFusedHeading();
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise
		// makes the angle increase.
		return 360.0 - imu.getYaw();
	}

    public double getRate() {
        return imu.getRate();
    }

    public void reset () {
        imu.reset();
    }
    
    public Rotation2d getRotation2d() {
        return imu.getRotation2d();
    }

    public Rotation3d getRotation3d() {
        return imu.getRotation3d();
    }

    public AHRS getIMU() {
        return imu;
    }

}
