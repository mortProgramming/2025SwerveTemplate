package frc.robot.MORTlib.hardware.brands.rev;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.MORTlib.hardware.motor.MotorIntf;

public class SparkFlexMotor implements MotorIntf {

    public int ID;
    public SparkLowLevel.MotorType brushType;

    public SparkFlex motor;
    public SparkMaxConfig config;
    public SparkClosedLoopController controller;

    // CANSparkLowLevel.MotorType.kBrushless
    public SparkFlexMotor(int ID, SparkLowLevel.MotorType brushType) {
        this.ID = ID;
        this.brushType = brushType;

        motor = new SparkFlex(ID, brushType);
        config = new SparkMaxConfig();
        controller = motor.getClosedLoopController();

        config.closedLoop.pid(0, 0, 0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setCurrentLimit(double limit) {
        config.secondaryCurrentLimit(limit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void setDirectionFlip(boolean direction) {
        config.inverted(direction);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setPIDValues(double kP, double kI, double kD) {
        config.closedLoop.pid(kP, kI, kD);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setPercent(double percent) {
        motor.set(percent);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setPositionRotations(double setpoint) {
        controller.setReference(setpoint, SparkFlex.ControlType.kMAXMotionPositionControl);
    }

    public void setCanivore(String canivore) {
        System.out.println("Why are you here?");
    }

    

    public double getPositionRotations() {
        return motor.getEncoder().getPosition();
    }

    public double getVelocityRPM() {
        return motor.getEncoder().getVelocity();
    }

    public double getOutputVoltage() {
        return motor.getAppliedOutput();
    }

    public SparkFlex getMotor() {
        return motor;
    }
}
