package frc.robot.subsystems;

import frc.robot.Constants.CANIDs;
import frc.robot.Constants.CANIDs.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase{
    SparkMax manipulator;
    LaserCan sensor;

    SparkBaseConfig manipulatorconfig = new SparkMaxConfig();

    public ManipulatorSubsystem() {
        manipulator = new SparkMax(CANIDs.kManipulatorID, MotorType.kBrushless);
        manipulatorconfig.inverted(false).idleMode(IdleMode.kBrake);
        manipulator.configure(manipulatorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sensor = new LaserCan(CANIDs.kLaserCANID);
    }

    public void setManipulatorVoltage(double volts) {
        manipulator.setVoltage(volts);
    }

    public double sensorMeasurement() {
        return sensor.getMeasurement().distance_mm;
    }

    public boolean hasCoral() {
        return sensorMeasurement() < 100;
    }

    public void periodic() {
        SmartDashboard.putBoolean("hasCoral", sensorMeasurement() < 100);
        SmartDashboard.putNumber("lasercan", sensorMeasurement());
        super.periodic();
    }
}
