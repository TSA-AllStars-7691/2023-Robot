// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.executeTrajectory;
import frc.robot.commands.AutoBalancing;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    /* Drive Controls */
    private static final int translationAxis = XboxController.Axis.kLeftY.value;
    private static final int strafeAxis = XboxController.Axis.kLeftX.value;
    private static final int rotationAxis = XboxController.Axis.kRightX.value;
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(Constants.OIConstants.kDriverController);
    private final CommandXboxController operator = new CommandXboxController(Constants.OIConstants.kOperatorController);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final GripperSubsystem m_gripper = new GripperSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    /* Autonomous Mode Chooser */
    private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

    /* Autonomous Modes */
    PathPlannerTrajectory moveForward = PathPlanner.loadPath("Move Forward",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    PathPlannerTrajectory sCurve = PathPlanner.loadPath("S Curve",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    PathPlannerTrajectory sussy = PathPlanner.loadPath("sussy",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private final TeleopSwerve t_TeleSwerve = new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            () -> true,
            () -> true);

    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                t_TeleSwerve);

        // Configure the button bindings
        configureButtonBindings();

        // Configure Smart Dashboard options
        configureSmartDashboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // <----------- Driver button mappings
        // driver.x().onTrue(new AutoBalancing(s_Swerve));
        driver.y().onTrue(new InstantCommand(s_Swerve::zeroGyro));

        // <----------- Operator button mappings

        // set up gripper open/close
        XboxController operatorHid = operator.getHID();
        operator.rightBumper()
                .onTrue(new InstantCommand(m_gripper::openGripper))
                .onFalse(new InstantCommand(m_gripper::closeGripper));
        operator.leftBumper()
                .onTrue(new InstantCommand(m_gripper::openGripper))
                .onFalse(new InstantCommand(m_gripper::closeGripperCube));
        driver.leftBumper()
                .onTrue(new InstantCommand(t_TeleSwerve::fastMode))
                .onFalse(new InstantCommand(t_TeleSwerve::MediumMode));
        driver.rightBumper()
                .onTrue(new InstantCommand(t_TeleSwerve::slowMode))
                .onFalse(new InstantCommand(t_TeleSwerve::MediumMode));

        // set up arm preset positions
        operator.a().onTrue(new InstantCommand(
                () -> m_arm.setTargetPosition(Constants.Arm.kHomePosition, m_gripper)));
        operator.x().onTrue(new InstantCommand(
                () -> m_arm.setTargetPosition(Constants.Arm.l2CubeScoringPostition, m_gripper)));
        operator.y().onTrue(new InstantCommand(
                () -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition, m_gripper)));
        operator.b().onTrue(new InstantCommand(
                () -> m_arm.setTargetPosition(Constants.Arm.l2ConeScoringPostition, m_gripper)));
        operator.povLeft().onTrue(new InstantCommand(
                () -> m_arm.setTargetPosition(Constants.Arm.ItemHoldPosition, m_gripper)));
        operator.povRight().onTrue(new InstantCommand(
                () -> m_arm.armSlowMode()))
                .onFalse(new InstantCommand(
                        () -> m_arm.armDefaultMode()));

        // set up arm manual and auto functions
        m_arm.setDefaultCommand(new RunCommand(m_arm::runAutomatic, m_arm));
        new Trigger(() -> Math.abs(operatorHid.getRightTriggerAxis()
                - operatorHid.getLeftTriggerAxis()) > Constants.OIConstants.kArmManualDeadband)
                .whileTrue(new RunCommand(
                        () -> m_arm.runManual((operatorHid.getRightTriggerAxis() - operatorHid.getLeftTriggerAxis())
                                * Constants.OIConstants.kArmManualScale),
                        m_arm));
    }

    private void configureSmartDashboard() {
        autoChooser.setDefaultOption("Move forward", moveForward);
        autoChooser.addOption("S curve", sCurve);
        autoChooser.addOption("SUSSY - CADEN", sussy);

        SmartDashboard.putData(autoChooser);
        SmartDashboard.putData(m_arm);
        SmartDashboard.putData(m_gripper);

    }

    public void disabledInit() {
        s_Swerve.resetToAbsolute();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Executes the autonomous command chosen in smart dashboard
        return new executeTrajectory(s_Swerve, autoChooser.getSelected());
    }
}
