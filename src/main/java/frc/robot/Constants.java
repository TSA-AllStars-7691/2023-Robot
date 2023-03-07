package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.PIDGains;
import java.lang.Math;
public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(17.25);
    public static final double wheelBase = Units.inchesToMeters(31.7);
    public static final double wheelDiameter = Units.inchesToMeters(3.95);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 6.12:1
    public static final double angleGearRatio = (( 150.0 / 7.0) / 1.0); // 150/7:0

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    public static final double pitchSetPoint = 0.0;

    public static final double drivePitchKP = 0.04;
    public static final double drivePitchKI = 0.00005;
    public static final double drivePitchKD = 0.000000000000001;
    public static final double drivePitchKFF = 0.000000000000001;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.005;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.0;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.11979;
    public static final double driveKV = 2.3823;
    public static final double driveKA = 0.30034;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 2; // meters per second
    public static final double maxAngularVelocity = 3;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 1;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180-0.527);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1.413);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 2;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180-39.287);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(141.328);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(143.350 );
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(142.998);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 4;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(54.053 );
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-126.211);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final PathConstraints constraints = new PathConstraints(1, 1);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
  }

  public static final class PhotonVision{
    public static final String photonVisionName = "OV5647";
    public static final Transform3d robotToCam =
    new Transform3d(
            new Translation3d(Units.inchesToMeters(11.4), 0.0, Units.inchesToMeters(6.4)),
            new Rotation3d(
                    0, 0,
                    0));
  }

  public static final class AprilTags {
    public static final AprilTag tag1 = new AprilTag(1, FieldConstants.aprilTags.get(1));
    public static final AprilTag tag2 = new AprilTag(2, FieldConstants.aprilTags.get(2));
    public static final AprilTag tag3 = new AprilTag(3, FieldConstants.aprilTags.get(3));
    public static final AprilTag tag4 = new AprilTag(4, FieldConstants.aprilTags.get(4));
    public static final AprilTag tag5 = new AprilTag(5, FieldConstants.aprilTags.get(5));
    public static final AprilTag tag6 = new AprilTag(6, FieldConstants.aprilTags.get(6));
    public static final AprilTag tag7 = new AprilTag(7, FieldConstants.aprilTags.get(7));
    public static final AprilTag tag8 = new AprilTag(8, FieldConstants.aprilTags.get(8));
    public static final ArrayList<AprilTag> aprilTagList = new ArrayList<>();

    static {
      aprilTagList.add(tag1);
      aprilTagList.add(tag2);
      aprilTagList.add(tag3);
      aprilTagList.add(tag4);
      aprilTagList.add(tag5);
      aprilTagList.add(tag6);
      aprilTagList.add(tag7);
      aprilTagList.add(tag8);
    }
  }

  public static final class Arm {
    public static final int kArmCanId = 5;
    public static final boolean kArmInverted = false;
    public static final int kCurrentLimit = 40;

    public static final double kSoftLimitReverse = 0.0;
    public static final double kSoftLimitForward = 4.6;

    public static final double kArmGearRatio = 1.0 / (48.0 * 4.0); 
    public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
    public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
    public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

    public static final double kHomePosition = 0.0;
    public static final double kScoringPosition = 3.05;
    public static final double kIntakePosition = 4.52;
    public static final double kFeederPosition = 2.95;
}

public static final class Gripper {
    public static final int kGripperCanId = 6;
    public static final double kSoftLimitReverse = -34.0;
    public static final double kSoftLimitForward = 5.0;
    public static final double kClosePosition = 0.0;
    public static final double kOpenPosition = -34.0;
    public static final double kSafePosition = -29.0;
    public static final int kCurrentLimit = 10;
    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);
}

}