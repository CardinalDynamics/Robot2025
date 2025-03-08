package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;

public class ClimberSubsystem extends SubsystemBase{
    SparkMax winch;
    // SparkMax climber;
    SparkMaxConfig winchConfig = new SparkMaxConfig();
    SparkMaxConfig climberConfig = new SparkMaxConfig();

    public ClimberSubsystem() {
        winch = new SparkMax(CANIDs.kWinchID, MotorType.kBrushless);
        // climber = new SparkMax(CANIDs.kClimberWheelsID, MotorType.kBrushless);
        winchConfig.inverted(false).idleMode(IdleMode.kBrake);
        climberConfig.inverted(false).idleMode(IdleMode.kBrake);

    }

    public void setWinchVoltage(double volts) {
        winch.setVoltage(volts);
    }

    // public void setClimberVoltage(double volts) {
    //     climber.setVoltage(volts);
    // }
}