package frc.robot.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class segmentLineUp {

    public static PathPlannerTrajectory getTrajectory(Constants.SEGMENT segment, Supplier<Pose2d> startPose) {
        return getTrajectory(segment, startPose, Constants.Autonomous.constraints);
    }

    /**
     * 
     * @param s_Swerve
     * @param segment    the segment TODO
     * @param startPoint
     */
    public static PathPlannerTrajectory getTrajectory(Constants.SEGMENT segment, Supplier<Pose2d> startPose, PathConstraints constraints) {

        Translation2d lineUpTranslation = startPose.get().getTranslation();
        Rotation2d lineUpRotation = startPose.get().getRotation();

        switch (segment) {
            case CONE_1:
                lineUpTranslation = new Translation2d(1.98, 4.93);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case CONE_2:
                lineUpTranslation = new Translation2d(1.98, 3.89);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case CONE_3:
                lineUpTranslation = new Translation2d(1.98, 3.25);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case CONE_4:
                lineUpTranslation = new Translation2d(1.98, 2.2);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case CONE_5:
                lineUpTranslation = new Translation2d(1.98, 1.6);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case CONE_6:
                lineUpTranslation = new Translation2d(1.98, .47);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case CUBE_1:
                lineUpTranslation = new Translation2d(1.98, 4.43);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case CUBE_2:
                lineUpTranslation = new Translation2d(1.98, 2.74);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case CUBE_3:
                lineUpTranslation = new Translation2d(1.98, 1.05);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case HUMANPLAYER:
                lineUpTranslation = new Translation2d(14.40, 7.59);
                lineUpRotation = Rotation2d.fromDegrees(90);
                break;
            case BLUE_DOUBLE_SUBSTATIOIN:
                lineUpTranslation = new Translation2d(16.178784 - Units.inchesToMeters(36.5), 
                    6.749796 - Units.inchesToMeters(28.5));
                lineUpRotation = Rotation2d.fromDegrees(0);
                break;
            case BLUE_CHARGE_UP:
                lineUpTranslation = new Translation2d(3.92, 2.71);
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case BLUE_BEFORE_DOUBLE_SUBSTATIOIN:
                lineUpTranslation = new Translation2d(16.178784 - Units.inchesToMeters(36.5 + 18), 
                    6.749796);
                lineUpRotation = Rotation2d.fromDegrees(0);
                break;
            case RED_DOUBLE_SUBSTATIOIN:
                lineUpTranslation = new Translation2d(0.36195 + Units.inchesToMeters(36.5), 
                    6.749796 - Units.inchesToMeters(28.5));
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
            case RED_BEFORE_DOUBLE_SUBSTATIOIN:
                lineUpTranslation = new Translation2d(0.36195 + Units.inchesToMeters(36.5 + 18), 
                    6.749796 );
                lineUpRotation = Rotation2d.fromDegrees(180);
                break;
                
        }

        Rotation2d startHeading = new Translation2d(lineUpTranslation.getX(), lineUpTranslation.getY())
                .minus(startPose.get().getTranslation()).getAngle();
        Rotation2d endHeading = new Translation2d(lineUpTranslation.getX(), lineUpTranslation.getY())
                .minus(startPose.get().getTranslation()).getAngle();

        PathPoint startPoint = new PathPoint(startPose.get().getTranslation(), startHeading,
                startPose.get().getRotation(), constraints.maxVelocity);
        PathPoint lineUpPoint = new PathPoint(lineUpTranslation, endHeading, lineUpRotation, constraints.maxVelocity);

        return PathPlanner.generatePath(
                constraints,
                startPoint,
                lineUpPoint);
    }
}