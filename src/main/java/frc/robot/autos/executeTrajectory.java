package frc.robot.autos;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class executeTrajectory extends SequentialCommandGroup {
  public executeTrajectory(Swerve s_Swerve, PathPlannerTrajectory trajectory) {
    PathPlannerTrajectory allianceTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory,
        DriverStation.getAlliance());

    s_Swerve.getField().getObject("Field").setTrajectory(allianceTrajectory);

    PIDController thetaController = new PIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        allianceTrajectory,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(allianceTrajectory.getInitialHolonomicPose())),
        swerveControllerCommand);
  }
}
