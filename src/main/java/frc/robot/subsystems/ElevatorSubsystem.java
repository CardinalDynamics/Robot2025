package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevator1;
    SparkMax elevator2;
    ProfiledPIDController controller;
    // PIDController controller;
    DutyCycleEncoder encoder;
    double targetPosition;
    SparkMaxConfig elevator1Config = new SparkMaxConfig();
    SparkMaxConfig elevator2Config = new SparkMaxConfig();


    public ElevatorSubsystem () {
        elevator1 = new SparkMax(1, MotorType.kBrushless);
        elevator2 = new SparkMax(2, MotorType.kBrushless);

        elevator1Config.inverted(false).idleMode(IdleMode.kBrake);
        elevator2Config.inverted(true).idleMode(IdleMode.kBrake);

        elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller = new ProfiledPIDController(0, 0, 0, null);

        encoder = new DutyCycleEncoder(0, 1.0, 0);

        targetPosition = 0.0;

        controller.setGoal(targetPosition);
        controller.setIZone(0);
    }

    public double getElevatorPosition() {
        return encoder.get();
    }

    public void setVolts(double volts) {
        elevator1.setVoltage(volts);
        elevator2.setVoltage(volts);
        controller.getSetpoint().position = getElevatorPosition();
    }

    public void setTargetPosition(double position){
        targetPosition = position;
        controller.setGoal(targetPosition);
    }

    public void usePIDOutput() {
        elevator1.setVoltage(controller.calculate(getElevatorPosition()));
        elevator2.setVoltage(controller.calculate(getElevatorPosition()));
    }

    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Setpoint", controller.getSetpoint().position);
        SmartDashboard.putNumber("Elevator Goal", controller.getGoal().position);
        SmartDashboard.updateValues();
    }

}
