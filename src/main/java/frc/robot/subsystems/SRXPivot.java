// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    AMP(45),
    SPEAKER(90),
    SOURCE(135);

    POSITION(double deg) {
      degrees = deg;
    }

    double degrees = 0;
  }

  public class PeriodicIO {
    public CONTROL_STATE controlState = CONTROL_STATE.OPEN_LOOP;
    public double targetPosition = 0;
    public double position = 0;
  }

  private PeriodicIO m_PeriodicIO = new PeriodicIO();

  /** Creates a new SRXPivot. */
  public SRXPivot() {
    m_pivotMaster.configFactoryDefault();
    m_pivotFollower.configFactoryDefault();

    m_pivotFollower.follow(m_pivotMaster);

    m_pivotMaster.setNeutralMode(NeutralMode.Brake);
    m_pivotFollower.setNeutralMode(NeutralMode.Brake);
    m_pivotMaster.setInverted(InvertType.None);
    m_pivotFollower.setInverted(InvertType.OpposeMaster);
    m_pivotMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));
    m_pivotFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));

    TalonSRXConfiguration pivotConfig = new TalonSRXConfiguration();
    pivotConfig.slot0.kP = 1023 / (angleToTicks(70));
    pivotConfig.slot0.kI = 0;
    pivotConfig.slot0.kD = 0;
    pivotConfig.slot0.kF = 0;
    pivotConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

    pivotConfig.motionCruiseVelocity = (int) (angleToTicks(25) * 0.1);
    pivotConfig.motionAcceleration = (int) (angleToTicks(25) * 0.1);

    pivotConfig.motionCurveStrength = 2;

    zero();
  }

  /*
   * FUNCTIONALITY
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read sensors
    m_PeriodicIO.position = m_pivotMaster.getSelectedSensorPosition();

  }

  public void zero() {
    m_pivotMaster.setSelectedSensorPosition(0);
  }

  public void goToPosition(POSITION pos) {
    if (m_PeriodicIO.controlState != CONTROL_STATE.MOTION_MAGIC) {
      m_PeriodicIO.controlState = CONTROL_STATE.MOTION_MAGIC;
    }
    m_PeriodicIO.targetPosition = angleToTicks(pos.degrees);
    m_pivotMaster.set(ControlMode.MotionMagic, m_PeriodicIO.targetPosition);
    System.out.println("go to position" + m_PeriodicIO.targetPosition);
  }

  private boolean isAtTarget() {
    return m_PeriodicIO.controlState == CONTROL_STATE.MOTION_MAGIC
        && ticksToAngle(Math.abs(m_PeriodicIO.position - m_PeriodicIO.targetPosition)) < 0.5;
  }

  public int angleToTicks(double angle) {
    return (int) (angle / 360.0 * m_gearRatio * 4096.0);
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
    builder.addDoubleProperty("current ticks: ", () -> ticksToAngle(m_PeriodicIO.position),
        null);
    builder.addDoubleProperty("target ticks: ", () -> ticksToAngle(m_PeriodicIO.targetPosition), null);
  }

  /*
   * COMMANDS
   */
  public Command defaultCmd() {
    // hold last
    return run(() -> stop());
  }

  public Command getGotoPositionCommand(POSITION pos) {
    return runOnce(() -> goToPosition(pos));
  }

  public Command getGotoAndWaitPositionCommand(POSITION pos) {
    return getGotoPositionCommand(pos)
        .alongWith(Commands.none().until(() -> isAtTarget()));
  }
}
