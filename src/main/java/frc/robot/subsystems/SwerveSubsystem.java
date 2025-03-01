package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;
    RobotConfig config;
    AHRS navx;
    Boolean isRobotOriented = false;
    
    public SwerveSubsystem(File directory) {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;

        // Make a swerve drive from json files
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Units.feetToMeters(4.5),
                new Pose2d(new Translation2d(), new Rotation2d(Math.toDegrees(0))));
            
        } catch(Exception e) {
            SmartDashboard.putString("error", e.getMessage());
        }

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
        }

        navx = (AHRS)swerveDrive.getGyro().getIMU();
        

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.001, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
        );
    }

    public void drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularVelocity) {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
            angularVelocity.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
            true,
            false);
    }

    public void driveSlow(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularVelocity) {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * .5,
            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * .5),
            angularVelocity.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity() * .5,
            true,
            false);
    }

    public void driveRobotOriented(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularVelocity) {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
            angularVelocity.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
            false,
            false);
    }

    public void driveRobotOrientedSlow(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularVelocity) {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * .5,
            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * .5),
            angularVelocity.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity() * .5,
            false,
            false);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    public void toggleDriveMode() {
        isRobotOriented = !isRobotOriented;
    }

    public boolean getDriveMode() {
        return isRobotOriented;
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("yaw", Math.toDegrees(swerveDrive.getGyroRotation3d().getAngle()));
        SmartDashboard.putNumber("autobuilderpose", AutoBuilder.getCurrentPose().getRotation().getDegrees());
        SmartDashboard.putNumber("swerveDrive pose", swerveDrive.getPose().getRotation().getDegrees());
        SmartDashboard.putBoolean("robot oriented", getDriveMode());
        // TODO Auto-generated method stub
        super.periodic();
    }

}