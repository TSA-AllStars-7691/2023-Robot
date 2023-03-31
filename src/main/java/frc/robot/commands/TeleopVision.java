package frc.robot.commands;

public class TeleopVision {

	
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
