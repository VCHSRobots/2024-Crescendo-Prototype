// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class SRXPivot extends SubsystemBase {
  private final WPI_TalonSRX m_pivotMaster = new WPI_TalonSRX(PivotConstants.kPivotMasterId.getDeviceNumber());
  private final WPI_TalonSRX m_pivotFollower = new WPI_TalonSRX(PivotConstants.kPivotFollowerId.getDeviceNumber());

  private final double m_gearRatio = (60.0 / 18.0);

  enum CONTROL_STATE {
    OPEN_LOOP,
    MOTION_MAGIC
  }

  public enum POSITION {
    HOME(0),
    AMP(90),
    SPEAKER(30),
    SPEAKER_CLOSE_AUTO(30),
    SPEAKER_MID_AUTO(50),
    SOURCE(90);

    POSITION(double deg) {
      degrees = deg;
    }

    double degrees = 0;
  }

  public class PeriodicIO {
    public CONTROL_STATE controlState = CONTROL_STATE.OPEN_LOOP;
    public double targetTicks = 0;
    public double positionTicks = 0;
  }

  private PeriodicIO m_PeriodicIO = new PeriodicIO();

  /** Creates a new SRXPivot. */
  public SRXPivot() {
    m_pivotMaster.configFactoryDefault();
    m_pivotFollower.configFactoryDefault();

    m_pivotMaster.setNeutralMode(NeutralMode.Brake);
    m_pivotFollower.setNeutralMode(NeutralMode.Brake);
    m_pivotMaster.setInverted(InvertType.None);
    m_pivotFollower.setInverted(InvertType.OpposeMaster);

    TalonSRXConfiguration pivotConfig = new TalonSRXConfiguration();
    pivotConfig.slot0.kP = 1023.0 / (angleToTicks(40));
    pivotConfig.slot0.kI = 0;
    pivotConfig.slot0.kD = 0;
    pivotConfig.slot0.kF = 0;
    pivotConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

    pivotConfig.voltageCompSaturation = 12;

    pivotConfig.motionCruiseVelocity = (int) (angleToTicks(55) * 0.1);
    pivotConfig.motionAcceleration = (int) (angleToTicks(75) * 0.1);

    pivotConfig.motionCurveStrength = 2;

    m_pivotMaster.configAllSettings(pivotConfig);
    m_pivotFollower.configAllSettings(pivotConfig);

    m_pivotMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));
    m_pivotFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));
    m_pivotMaster.enableVoltageCompensation(true);
    m_pivotFollower.enableVoltageCompensation(true);
    m_pivotFollower.follow(m_pivotMaster);

    zero();
  }

  /*
   * FUNCTIONALITY
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read sensors
    m_PeriodicIO.positionTicks = m_pivotMaster.getSelectedSensorPosition();

  }

  public void zero() {
    m_pivotMaster.setSelectedSensorPosition(0);
  }

  public void setTargetDegrees(double degrees) {
    if (m_PeriodicIO.controlState != CONTROL_STATE.MOTION_MAGIC) {
      m_PeriodicIO.controlState = CONTROL_STATE.MOTION_MAGIC;
    }
    m_PeriodicIO.targetTicks = angleToTicks(degrees);
    m_pivotMaster.set(ControlMode.MotionMagic, m_PeriodicIO.targetTicks, DemandType.ArbitraryFeedForward,
        getFeedForward());
  }

  public void setTargetDegreesToCurrentPosition() {
    setTargetDegrees(ticksToAngle(m_PeriodicIO.positionTicks));
  }

  private boolean isAtTarget() {
    return m_PeriodicIO.controlState == CONTROL_STATE.MOTION_MAGIC
        && ticksToAngle(Math.abs(m_PeriodicIO.positionTicks - m_PeriodicIO.targetTicks)) < 0.5;
  }

  public int angleToTicks(double angle) {
    return (int) (angle * 4096.0 * m_gearRatio / 360.0);
  }

  public double ticksToAngle(double ticks) {
    return ticks / 4096.0 / m_gearRatio * 360.0;
  }

  public void set(double percentOuput) {
    if (m_PeriodicIO.controlState != CONTROL_STATE.OPEN_LOOP) {
      m_PeriodicIO.controlState = CONTROL_STATE.OPEN_LOOP;
    }
    m_pivotMaster.set(percentOuput);
  }

  public void stop() {
    if (m_PeriodicIO.controlState != CONTROL_STATE.OPEN_LOOP) {
      m_PeriodicIO.controlState = CONTROL_STATE.OPEN_LOOP;
    }
    m_pivotMaster.set(0);
  }

  public double getFeedForward() {
    return Math.cos(ticksToAngle(m_PeriodicIO.positionTicks)) * .10;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("velocity output: ", () -> m_pivotMaster.getSelectedSensorVelocity(),
        null);
    builder.addDoubleProperty("supply current: ", () -> m_pivotMaster.getSupplyCurrent(),
        null);
    builder.addDoubleProperty("stator current: ", () -> m_pivotMaster.getStatorCurrent(),
        null);
    builder.addDoubleProperty("voltage: ", () -> m_pivotMaster.getMotorOutputVoltage(), null);
    builder.addDoubleProperty("current angle: ", () -> ticksToAngle(m_PeriodicIO.positionTicks),
        null);

    builder.addDoubleProperty("target angle: ", () -> ticksToAngle(m_PeriodicIO.targetTicks), null);
    builder.addDoubleProperty("closed loop error", () -> m_pivotMaster.getClosedLoopError(), null);
    builder.addBooleanProperty("is at target", () -> isAtTarget(), null);
  }

  /*
   * COMMANDS
   */
  public Command getHoldPositionCommand() {
    return run(() -> setTargetDegrees(m_PeriodicIO.targetTicks));
  }

  public Command getGotoPositionCommand(POSITION pos) {
    return run(() -> setTargetDegrees(pos.degrees));
  }

  public Command getGotoPositionUntilTargetCommand(POSITION pos) {
    return getGotoPositionCommand(pos).until(this::isAtTarget);
  }
}
