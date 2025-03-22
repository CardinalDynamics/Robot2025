package frc.robot.subsystems;

import frc.robot.Constants.CANIDs;
import frc.robot.Constants.CANIDs.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevator1;
    SparkMax elevator2;
    ProfiledPIDController controller;
    Encoder encoder;
    double targetPosition;
    SparkMaxConfig elevator1Config = new SparkMaxConfig();
    SparkMaxConfig elevator2Config = new SparkMaxConfig();
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.15, 0);

    public final double kL2SETPOINT = 2475.0;
    public final double kL3SETPOINT = 5250.0;
    public final double kL4SETPOINT = 9600.0;


    public ElevatorSubsystem () {
        elevator1 = new SparkMax(CANIDs.kElevator1ID, MotorType.kBrushless);
        elevator2 = new SparkMax(CANIDs.kElevator2ID, MotorType.kBrushless);

        elevator1Config.inverted(true).idleMode(IdleMode.kBrake);
        elevator2Config.inverted(false).idleMode(IdleMode.kBrake);

        elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // .025
        controller = new ProfiledPIDController(0.01, 0.0, 0.0, new Constraints(14000.0, 14000.0));

        encoder = new Encoder(0, 1, false);
        encoder.reset();

        targetPosition = 0.0;

        controller.setGoal(targetPosition);
        controller.setTolerance(30.0);
    }

    public double getElevatorPosition() {
        return encoder.getDistance();
    }

    public double getElevatorRate() {
        return encoder.getRate();
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
        elevator1.setVoltage(controller.calculate(getElevatorPosition()) + feedforward.calculate(0));
        elevator2.setVoltage(controller.calculate(getElevatorPosition()) + feedforward.calculate(0));
    }

    public boolean atGoal() {
        return controller.atGoal();
    }

    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Setpoint", controller.getSetpoint().position);
        SmartDashboard.putNumber("Elevator Goal", controller.getGoal().position);
        SmartDashboard.putBoolean("atSetpoint", atGoal());
    }

}
