// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.VariableAutos.BranchSide;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.IntakeAuto;
import frc.robot.commands.ScoreL4WithSensor;
import frc.robot.commands.ScoreSecondL4;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerver;
  ElevatorSubsystem elevator = new ElevatorSubsystem();
  ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  FunnelSubsystem funnel = new FunnelSubsystem();
  LEDSubsystem leds = new LEDSubsystem();
  PathPlannerPath path;
  AutoAlign alignmentCommandFactory;
  Trigger robotOriented;
  Trigger hasCoral;
  Trigger endGame;

  private Field2d m_field = new Field2d();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  public final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("intake coral", new IntakeAuto(manipulator, funnel).withTimeout(4)
        .andThen(Commands.runOnce(() -> manipulator.setManipulatorVoltage(0))));
    NamedCommands.registerCommand("score L4", new ScoreL4WithSensor(manipulator, elevator));
    NamedCommands.registerCommand("second L4", new ScoreSecondL4(manipulator, elevator));
    swerver = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    // Load path
    loadPaths();
    robotOriented = new Trigger(() -> swerver.getDriveMode());
    hasCoral = new Trigger(() -> manipulator.hasCoral());
    endGame = new Trigger(() -> DriverStation.getMatchTime() < 20.0);
    SmartDashboard.putData("Field", m_field);

    m_autoChooser.setDefaultOption("Leave", AutoBuilder.buildAuto("Start2-Leave"));
    m_autoChooser.addOption("Left 3 Coral", AutoBuilder.buildAuto("Start1-test"));
    m_autoChooser.addOption("Right 3 Coral", AutoBuilder.buildAuto("Start3-test"));
    m_autoChooser.addOption("Barge Wall 3 Coral", AutoBuilder.buildAuto("Start0-test"));
    m_autoChooser.addOption("Processor Wall 3 Coral", AutoBuilder.buildAuto("Start5-test"));
    m_autoChooser.addOption("straight 1 coral", AutoBuilder.buildAuto("Start2-score"));
    m_autoChooser.addOption("scorel4-test", AutoBuilder.buildAuto("L4Score-test"));
    m_autoChooser.addOption("Straight", AutoBuilder.buildAuto("Straight"));
    m_autoChooser.addOption("none", Commands.waitSeconds(0));
    m_autoChooser.addOption("testtests", AutoBuilder.buildAuto("Test"));
    SmartDashboard.putData("chooser", m_autoChooser);

    alignmentCommandFactory = new AutoAlign(swerver);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    elevator.setDefaultCommand(Commands.run(() -> elevator.setVolts(0), elevator));
    manipulator.setDefaultCommand(Commands.run(() -> manipulator.setManipulatorVoltage(0), manipulator));
    funnel.setDefaultCommand(Commands.run(() -> funnel.setFunnelVoltage(0), funnel));

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      robotOriented.whileTrue(Commands.run(() -> swerver.driveRobotOriented(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
        () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .15)), swerver))
        .whileFalse(Commands.run(() -> swerver.drive(
          () -> MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
          () -> MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
          () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .15)), swerver));
      
      swerver.setDefaultCommand(Commands.run(() -> swerver.drive(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
        () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .15)), swerver));
    } else {
      robotOriented.whileTrue(Commands.run(() -> swerver.driveRobotOriented(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
        () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .15)), swerver))
        .whileFalse(Commands.run(() -> swerver.drive(
          () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
          () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
          () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .15)), swerver));

          swerver.setDefaultCommand(Commands.run(() -> swerver.drive(
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), .1),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), .1),
            () -> -.9 * MathUtil.applyDeadband(m_driverController.getRightX(), .15)), swerver));
    }

    m_driverController.rightBumper().onTrue(Commands.runOnce(() -> swerver.toggleDriveMode(), swerver));
    m_driverController.a().onTrue(Commands.runOnce(() -> swerver.zeroGyro()));

    // m_driverController.y().whileTrue(AutoBuilder.followPath(path));

    m_driverController.x().whileTrue(alignmentCommandFactory.generateCommand(BranchSide.LEFT));
    m_driverController.b().whileTrue(alignmentCommandFactory.generateCommand(BranchSide.RIGHT));
    m_driverController.y().whileTrue(alignmentCommandFactory.generateCommand(BranchSide.CENTER));

    m_operatorController.rightBumper().whileTrue(Commands.run(() -> elevator.setVolts(2), elevator));
    m_operatorController.leftBumper().whileTrue(Commands.run(() -> elevator.setVolts(-2), elevator));

    m_operatorController.rightTrigger()
        .whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(4), manipulator));
    m_operatorController.rightTrigger().whileTrue(Commands.run(() -> funnel.setFunnelVoltage(-4.0), funnel));

    m_operatorController.leftTrigger()
        .whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(-4), manipulator));
    m_operatorController.leftTrigger().whileTrue(Commands.run(() -> funnel.setFunnelVoltage(4), funnel));

    m_operatorController.y().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL3SETPOINT)));
    m_operatorController.y().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    m_operatorController.x().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL2SETPOINT)));
    m_operatorController.x().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    m_operatorController.b().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL4SETPOINT)));
    m_operatorController.b().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    m_operatorController.a().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(10.0)));
    m_operatorController.a().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));
    
    m_operatorController.povLeft().onTrue(Commands.runOnce(() -> elevator.setTargetPosition(1200.0)));
    m_operatorController.povLeft().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    m_operatorController.povDown().onTrue(new IntakeCoral(manipulator, funnel, leds));
    // m_operatorController.leftBumper().whileTrue(Commands.run(() ->
    // climber.setClimberVoltage(-6), climber)).onFalse(Commands.runOnce(() ->
    // climber.setClimberVoltage(0), climber));
    m_operatorController.povUp().whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(10), manipulator));

    m_driverController.povDown().whileTrue(swerver.sysIdDriveMotorCommand());

    m_driverController.rightTrigger()
    .whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(4), manipulator));
    m_driverController.rightTrigger().whileTrue(Commands.run(() -> funnel.setFunnelVoltage(-4.0), funnel));

    hasCoral.onTrue(Commands.runOnce(() -> leds.setCoralState(), leds)).onFalse(Commands.runOnce(() -> leds.setDefaultLights(), leds));
    endGame.whileTrue(leds.runPattern());
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
    return m_autoChooser.getSelected();
  }
}
