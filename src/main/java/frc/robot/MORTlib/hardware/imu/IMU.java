package frc.robot.MORTlib.hardware.imu;

import frc.robot.MORTlib.hardware.brands.ctre.Pigeon2IMU;
import frc.robot.MORTlib.hardware.brands.kauailabs.NavX2IMU;
import frc.robot.MORTlib.hardware.brands.kauailabs.NavXIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class IMU implements IMUIntf {

    public IMUTypeEnum imuType;
    public int ID;

    public IMUIntf imu;

    public IMU(IMUTypeEnum imuType) {
        this(imuType, 0);
    }

    public IMU(IMUTypeEnum imuType, int ID) {
        this.imuType = imuType;
        this.ID = ID;

        switch (imuType) {
            case PIGEON2:
                imu = new Pigeon2IMU(ID);
                break;
            case NAVX2:
                imu = new NavX2IMU(ID);
                break;
            case NAVX:
                imu = new NavXIMU(ID);
                break;
        }
    }

    public void setCanivore(String canivore) {
        imu.setCanivore(canivore);
    }

    public double getAngle() {
        return imu.getAngle();
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

    public IMUTypeEnum getIMUType() {
        return imuType;
    }

}