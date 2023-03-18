// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_controller;
  private double m_setpoint;
  private double m_prevSetpoint;

  /** Creates a new ExampleSubsystem. */
  public GripperSubsystem() {
    m_motor = new CANSparkMax(Constants.Gripper.kGripperCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(Constants.Gripper.kCurrentLimit);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Gripper.kSoftLimitForward);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Gripper.kSoftLimitReverse);

    m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Gripper.kPositionPIDGains);

    m_motor.burnFlash();

    m_setpoint = Constants.Gripper.kClosePosition;
  }

  public boolean isSafe() {
    return m_encoder.getPosition() > Constants.Gripper.kSafePosition;
  }

  public void openGripper() {
    m_setpoint = Constants.Gripper.kOpenPosition;
  }

  public void closeGripper() {
    m_setpoint = Constants.Gripper.kClosePosition;
  }

  // TODO: add granular trigger controls for position/angle
  //       We can copy what is being done within the {@link frc.robot.subsystems.ArmSubsystem} to track
  //       a target position using the {@link edu.wpi.first.math.trajectory.TrapezoidProfile} class.

  @Override
  public void periodic() { // This method will be called once per scheduler run
    if (m_setpoint != m_prevSetpoint) {
      m_controller.setReference(m_setpoint, CANSparkMax.ControlType.kPosition);
    }
    m_prevSetpoint = m_setpoint;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    builder.addDoubleProperty("Position", () -> m_encoder.getPosition(), null);
  }
}