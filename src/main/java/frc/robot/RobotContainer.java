// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerver = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  ElevatorSubsystem elevator = new ElevatorSubsystem();
  ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  PathPlannerPath path;
  Trigger robotOriented = new Trigger(swerver::getDriveMode);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
    private final CommandXboxController m_operatorController =
      new CommandXboxController(1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Load path
    loadPaths();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    elevator.setDefaultCommand(Commands.run(() -> elevator.setVolts(0), elevator));
    manipulator.setDefaultCommand(Commands.run(() -> manipulator.setManipulatorVoltage(0), manipulator));

    swerver.setDefaultCommand(Commands.run(() -> swerver.drive(
      () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
      () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
      () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .1)), swerver));
    
    robotOriented.whileTrue(Commands.run(() -> swerver.driveRobotOriented(
      () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
      () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
      () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .1)), swerver));

    m_driverController.rightBumper().onTrue(Commands.runOnce(() -> swerver.toggleDriveMode()));
    m_driverController.a().onTrue(Commands.runOnce(() -> swerver.zeroGyro()));

    m_driverController.leftBumper().whileTrue(Commands.run(() -> swerver.driveSlow(
      () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
      () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
      () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .1)), swerver));

    m_driverController.leftBumper().and(robotOriented).whileTrue(Commands.run(() -> swerver.driveRobotOrientedSlow(
      () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
      () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
      () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .1)), swerver));

    m_driverController.y().whileTrue(AutoBuilder.followPath(path));

    
    m_operatorController.rightBumper().whileTrue(Commands.run(() -> elevator.setVolts(2), elevator));
    m_operatorController.leftBumper().whileTrue(Commands.run(() -> elevator.setVolts(-2), elevator));

    m_operatorController.rightTrigger().whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(
      MathUtil.applyDeadband(m_operatorController.getRightTriggerAxis(), .1) * 12.0), manipulator));
    
    m_operatorController.leftTrigger().whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(-2), manipulator));

    m_operatorController.y().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL3SETPOINT)));
    m_operatorController.y().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    m_operatorController.x().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL2SETPOINT)));
    m_operatorController.x().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    m_operatorController.b().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL4SETPOINT)));
    m_operatorController.b().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    m_operatorController.a().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(0)));
    m_operatorController.a().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

  }

  private void loadPaths() {
    try {
      path = PathPlannerPath.fromPathFile("Example Path");
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new WaitCommand(1);
  }
}
