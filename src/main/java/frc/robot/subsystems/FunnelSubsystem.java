package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;

public class FunnelSubsystem extends SubsystemBase{
    SparkMax funnel;
    SparkMaxConfig funnelConfig = new SparkMaxConfig();
    LaserCan funnelSensor;
    SparkMax roller;
    SparkMaxConfig rollerConfig = new SparkMaxConfig();

    public FunnelSubsystem() {
        funnel = new SparkMax(CANIDs.kFunnelID, MotorType.kBrushed);
        funnelConfig.inverted(false).idleMode(IdleMode.kBrake);
        funnel.configure(funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        funnelSensor = new LaserCan(CANIDs.kLaserCAN2ID);
        roller = new SparkMax(CANIDs.kRollerID, MotorType.kBrushless);
        rollerConfig.inverted(true).idleMode(IdleMode.kCoast);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setFunnelVoltage(double volts) {
        funnel.setVoltage(volts);
        roller.setVoltage(volts * .5);
    }

    public double sensorMeasurementFunnel() {
        var sensorMeasurementFunnel = funnelSensor.getMeasurement();
        if (sensorMeasurementFunnel != null) {
            return sensorMeasurementFunnel.distance_mm;
        }
        return 0;
    }

    public boolean elevatorBlocked() {
        return sensorMeasurementFunnel() < 100;
    }

    public void periodic() {
        SmartDashboard.putBoolean("elevatorBlocked", elevatorBlocked());
        SmartDashboard.putNumber("lasercan", sensorMeasurementFunnel());
        super.periodic();
    }
}