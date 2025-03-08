package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.VariableAutos.BranchSide;
import frc.robot.VariableAutos.ReefSide;
import frc.robot.commands.PositionPIDCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign {
    
    private final SwerveSubsystem swerve;

    public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();
    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public final Time kAlignmentAdjustmentTimeout = Seconds.of(0.075);

    public AutoAlign(SwerveSubsystem inSwerve) {
        swerve = inSwerve;

        Arrays.stream(AprilTagRegion.kReef.blue()).forEach((i) -> {
            fieldLayout.getTagPose(i).ifPresent((p) -> {
                blueReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.red()).forEach((i) -> {
            fieldLayout.getTagPose(i).ifPresent((p) -> {
                redReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.both()).forEach((i) -> {
            fieldLayout.getTagPose(i).ifPresent((p) -> {
                allReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });
    }

    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("desired branch", Pose2d.struct).publish();

    public Command generateCommand(BranchSide side) {
        return Commands.defer(() -> {
            var branch = getClosestBranch(side, swerve);
            desiredBranchPublisher.accept(branch);
    
            return getPathFromWaypoint(getWaypointFromBranch(branch));
        }, Set.of());
    }


    public Command generateCommand(final ReefSide reefTag, BranchSide side) {
        return Commands.defer(() -> {
            var branch = getBranchFromTag(reefTag.getCurrent(), side);
            desiredBranchPublisher.accept(branch);
    
            return getPathFromWaypoint(getWaypointFromBranch(branch));
        }, Set.of());
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(swerve.getPose().getTranslation(), getPathVelocityHeading(swerve.getFieldRelativeSpeeds(), waypoint)),
            waypoint
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return 
            Commands.sequence(
                Commands.print("start position PID loop"),
                PositionPIDCommand.generateCommand(swerve, waypoint, kAlignmentAdjustmentTimeout),
                Commands.print("end position PID loop")
            );
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            new PathConstraints(1.5, 3.0, 540.0, 7.20),
            new IdealStartingState(getVelocityMagnitude(swerve.getFieldRelativeSpeeds()), swerve.getPose().getRotation()), 
            new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path).andThen(
            Commands.print("start position PID loop"),
            PositionPIDCommand.generateCommand(swerve, waypoint, kAlignmentAdjustmentTimeout),
            Commands.print("end position PID loop")
        );
    }
    

    /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(swerve.getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

    /**
     * 
     * @return Pathplanner waypoint with direction of travel away from the associated reef side
     */
    private Pose2d getWaypointFromBranch(Pose2d branch){
        return new Pose2d(
            branch.getTranslation(),
            branch.getRotation().rotateBy(Rotation2d.k180deg)
        );
    }

    /**
     * 
     * @return target rotation for the robot when it reaches the final waypoint
     */
    private Rotation2d getBranchRotation(SwerveSubsystem swerve){
        return getClosestReefAprilTag(swerve.getPose()).getRotation().rotateBy(Rotation2d.k180deg);
    }

    public static Pose2d getClosestBranch(BranchSide side, SwerveSubsystem swerve){
        Pose2d tag = getClosestReefAprilTag(swerve.getPose());
        
        return getBranchFromTag(tag, side);
    }


    private static Pose2d getBranchFromTag(Pose2d tag, BranchSide side) {
        var translation = tag.getTranslation().plus(
            new Translation2d(
                side.tagOffset.getY(),
                side.tagOffset.getX() * (side == BranchSide.LEFT ? -1 : 1)
            ).rotateBy(tag.getRotation())
        );

        return new Pose2d(
            translation.getX(),
            translation.getY(),
            tag.getRotation()
        );
    }
    
    /**
     * get closest reef april tag pose to given position
     * 
     * @param pose field relative position
     * @return
     */
    public static Pose2d getClosestReefAprilTag(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        
        ArrayList<Pose2d> reefPoseList;
        if (alliance.isEmpty()) {
            reefPoseList = allReefTagPoses;
        } else{
            reefPoseList = alliance.get() == Alliance.Blue ? 
                blueReefTagPoses :
                redReefTagPoses;
        }


        return pose.nearest(reefPoseList);

    }

}