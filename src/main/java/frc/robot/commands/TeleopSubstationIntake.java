package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.autos.executeTrajectory;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.autos.segmentLineUp;


public class TeleopSubstationIntake extends SequentialCommandGroup {
	   //get cone from double substation autonomously during teleop.
	
	 public TeleopSubstationIntake(GripperSubsystem m_gripper, ArmSubsystem m_arm, SwerveAutoBuilder autoBuilder, Supplier<Pose2d> startPose) {
		DriverStation.Alliance alliance = DriverStation.getAlliance();
		// TODO: Create a Red and Blue version of these constants
		Constants.SEGMENT stopBefore = (alliance == DriverStation.Alliance.Blue) ? 
			Constants.SEGMENT.BLUE_BEFORE_DOUBLE_SUBSTATIOIN : 
			Constants.SEGMENT.BLUE_BEFORE_DOUBLE_SUBSTATIOIN;
		Constants.SEGMENT stopForIntake = (alliance == DriverStation.Alliance.Blue) ? 
			Constants.SEGMENT.RED_DOUBLE_SUBSTATIOIN : 
			Constants.SEGMENT.RED_DOUBLE_SUBSTATIOIN;

		addCommands(
                                
				autoBuilder.followPath(segmentLineUp.getTrajectory(stopBefore, startPose)),
				new InstantCommand( () -> m_arm.setTargetPosition(Constants.Arm.kDoubleSubstationPosition, m_gripper)),
				new InstantCommand(m_gripper::openGripper),
				autoBuilder.followPath(segmentLineUp.getTrajectory(stopForIntake, startPose)),
				new InstantCommand(m_gripper::closeGripper),
				new InstantCommand( () -> m_arm.setTargetPosition(Constants.Arm.kHomePosition, m_gripper))
		);
	}

	
	/*
	 * commands:
	 * 	conservative scenario 
	 * 	1: button press to line up at double substation
	 * 	2: button press to stay parallel with grid when scoring
	 * 		or 
	 * 	3: auto score L2 cone and leave community zone.
	 * 	best case scenario
	 * 	-: all listed above and...
	 * 	4: button press to deliver cone or cube to grid
	 * 		-robot must first align itself with the section of the grid it wants to score in by positioning the april
	 * 			tag of the grid section in the center of camera view or by making only one april tag visible to the 
	 * 			camera by coming close to the grid.
	 * 		-once the grid section is selected then the operator must select....
	 * ------------------------
	 *		Command 1 Steps
	 		-:swerve: Align with the april tag as to be directly in front of pre determine pick up point at one of the two 
				double substations, however be far enough away so that the arm can come out
			-:arm: bring arm to set position
			-:gripper: open gripper
			-:swerve: move forward
			-:gripper: close gripper
			-:serve: move back slightly 
			-:arm: bring to inside position
	
	 * 
	 */


}
