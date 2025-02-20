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

    
    public SwerveSubsystem(File directory) {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Make a swerve drive from json files
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Units.feetToMeters(4.5));
            
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
        navx.reset();
        swerveDrive.setGyroOffset(new Rotation3d(0, 0, Math.toRadians(90)));
        // swerveDrive.resetOdometry(new Pose2d());

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.001, 0.0, 0.0), // Translation PID constants
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("yaw", navx.getYaw());
        // TODO Auto-generated method stub
        super.periodic();
    }

}