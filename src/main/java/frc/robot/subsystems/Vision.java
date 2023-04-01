package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator positionEstimation;
    private AprilTagFieldLayout aprilTagLayout;

    /**
     * TODO
     */
    public Vision() {
        camera = new PhotonCamera(Constants.PhotonVision.photonVisionName);
        try {
            aprilTagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            positionEstimation = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP, camera,
                    Constants.PhotonVision.robotToCam);

            positionEstimation.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            positionEstimation = null;
        }
    }

    /**
     * 
     * @param prevEstimatedRobotPose
     * @return Optional<EstimatedRobotPose>
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (positionEstimation == null) {
            return Optional.empty();
        }
        positionEstimation.setReferencePose(prevEstimatedRobotPose);
        return positionEstimation.update();
    }

    /**
     * 
     */
    @Override
    public void periodic() {
        // Sets the april tag positions depending on which side the robot starts on.
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            positionEstimation.getFieldTags().setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } else {
            positionEstimation.getFieldTags().setOrigin(OriginPosition.kRedAllianceWallRightSide);
        }

        if (camera.getLatestResult().getBestTarget() != null) {
            try {
                PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
                SmartDashboard.putNumber("X From AprilTag", target.getBestCameraToTarget().getX());
                SmartDashboard.putNumber("Y From AprilTag", target.getBestCameraToTarget().getY());
                SmartDashboard.putNumber("Angle From AprilTag", target.getYaw());
            } catch (Exception e) {
                System.out.print("Invalid AprilTag detected");
            }
        }
    }

}