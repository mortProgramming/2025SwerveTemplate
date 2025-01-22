package frc.robot.MORTlib.hardware.brands.rev;

import com.revrobotics.spark.SparkFlex;

import frc.robot.MORTlib.hardware.motor.MotorIntf;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

public class CANSparkFlexMotor implements MotorIntf {

    public int ID;
    public SparkLowLevel.MotorType brushType;

    public SparkFlex motor;
    public SparkMaxConfig config;
    public SparkClosedLoopController controller;

    // CANSparkLowLevel.MotorType.kBrushless
    public CANSparkFlexMotor(int ID, SparkLowLevel.MotorType brushType) {
        this.ID = ID;
        this.brushType = brushType;

        motor = new SparkFlex(ID, brushType);
        config = new SparkMaxConfig();
        controller = motor.getClosedLoopController();

        config.closedLoop.pid(0, 0, 0);
        motor.configure(config);
    }

    public void setCurrentLimit(double limit) {
        motor.setSecondaryCurrentLimit(limit);
    }

    public void setDirectionFlip(boolean direction) {
        motor.setInverted(direction);
    }

    public void setPIDValues(double kP, double kI, double kD) {
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public void setPercent(double percent) {
        motor.set(percent);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setPositionRotations(double setpoint) {
        controller.setReference(setpoint, CANSparkFlex.ControlType.kSmartMotion);
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
