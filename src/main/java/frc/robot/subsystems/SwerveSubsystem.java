package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
// drive p0.0020645
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
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
        
        int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

        // Make a swerve drive from json files
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Units.feetToMeters(13.0),
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
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        // SwerveDrive.replaceSwerveModuleFeedforward(.07, 2.5, .6);

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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

    public void driveRobotOriented(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularVelocity) {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * .1,
            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * .1),
            angularVelocity.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity() * .1,
            false,
            false);
    }

    public Pose2d getPose() {
        return swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return swerveDrive.getFieldVelocity();
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

    public void lockWheels() {
        swerveDrive.lockPose();
    }

    public double getSpeed() {
        ChassisSpeeds fieldVelocity = getFieldRelativeSpeeds();
        return Math.sqrt(fieldVelocity.vxMetersPerSecond * fieldVelocity.vxMetersPerSecond + fieldVelocity.vyMetersPerSecond * fieldVelocity.vyMetersPerSecond);
    }

    public Command sysIdDriveMotorCommand() {
        SwerveDriveTest.angleModules(swerveDrive, new Rotation2d());
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, false),
            3.0, 5.0, 3.0);
    }

    @Override
    public void periodic() {
        
        LimelightHelpers.SetRobotOrientation("", swerveDrive.getPose().getRotation().getDegrees(), 0, 0.0, 0.0, 0.0, 0.0);
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        if (mt2 != null && mt2.tagCount > 0) {
            swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose,
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").timestampSeconds);
        }

        swerveDrive.updateOdometry();

        
        SmartDashboard.putData("Field", swerveDrive.field);
        SmartDashboard.putNumber("yaw", navx.getYaw());
        SmartDashboard.putNumber("autobuilderpose", AutoBuilder.getCurrentPose().getRotation().getDegrees());
        SmartDashboard.putNumber("swerveDrive pose", swerveDrive.getPose().getRotation().getDegrees());
        SmartDashboard.putBoolean("robot oriented", isRobotOriented);
        SmartDashboard.putNumber("pose x", swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("pose y", swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition().getY());
        // TODO Auto-generated method stub
        super.periodic();
    }

}