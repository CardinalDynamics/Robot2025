
package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

public class PositionPIDCommand extends Command{
    
    public SwerveSubsystem swerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    );

    public final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
    public final Distance kPositionTolerance = Inches.of(0.4);
    public final LinearVelocity kSpeedTolerance = InchesPerSecond.of(0.25);
    public static final Time kEndTriggerDebounce = Seconds.of(0.04);

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();

    private PositionPIDCommand(SwerveSubsystem swerve, Pose2d goalPose) {
        this.swerve = swerve;
        this.goalPose = goalPose;

        Pose2d diff = swerve.getPose().relativeTo(goalPose);

        boolean rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            kRotationTolerance.getRotations(), 
            0.0, 
            1.0
        );

        boolean position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

        boolean speed = swerve.getSpeed() < kSpeedTolerance.in(MetersPerSecond);

        System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);

        endTrigger = new Trigger(() -> {
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.driveRobotRelative(new ChassisSpeeds(0,0,0));
            swerve.lockWheels();
        });
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        swerve.driveRobotRelative(
            mDriveController.calculateRobotRelativeSpeeds(
                swerve.getPose(), goalState
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}
