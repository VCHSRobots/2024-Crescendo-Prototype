// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
 */
public class Pivot extends SubsystemBase {

  private TalonFX m_pivot = new TalonFX(Constants.kPivotMotor.getDeviceNumber(), Constants.kPivotMotor.getBus());
  private MotionMagicExpoVoltage m_positionVoltageRequest = new MotionMagicExpoVoltage(0);

  enum CONTROL_STATE {
    OPEN_LOOP,
    MOTION_MAGIC
  }

  public class PeriodicIO {
    public CONTROL_STATE controlState = CONTROL_STATE.OPEN_LOOP;
    public double targetPosition = 0;
    public double position = 0;
  }

  private PeriodicIO m_PeriodicIO = new PeriodicIO();

  /** Creates a new Pivot. */
  public Pivot() {
    var configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    configs.Feedback.SensorToMechanismRatio = 2; // TODO: sensor rot per output

    configs.CurrentLimits.SupplyCurrentLimit = 30;
    configs.CurrentLimits.SupplyCurrentThreshold = 30;
    configs.CurrentLimits.SupplyTimeThreshold = 0;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    var slot0 = configs.Slot0; // TODO pid sva tuning
    slot0.kS = 0; // Voltage to overcome static friction
    slot0.kV = 0; // Voltage per unit of target velocity (V/rps)
    slot0.kA = 0; // Voltage per unit of target acceleration (V/(rps/s))
    slot0.kP = 0; // V / rotation error
    slot0.kI = 0; // V / (error * s)
    slot0.kD = 0; // V / (error/s)
    var mmConfigs = configs.MotionMagic;
    mmConfigs.MotionMagicCruiseVelocity = 30; // peak velocity of the profile; set to 0 to target the system’s max
                                              // velocity
    mmConfigs.MotionMagicExpo_kA = 8; // voltage required to maintain a given velocity, in V/rps
    mmConfigs.MotionMagicExpo_kV = 7.2; // voltage required to apply a given acceleration, in V/(rps/s)

    m_pivot.getConfigurator().apply(configs);
    zero();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_PeriodicIO.position = m_pivot.getPosition().getValueAsDouble();
  }

  public void zero() {
    m_pivot.setPosition(0);
  }

  public enum POSITION {
    HOME(0),
    AMP(0),
    SPEAKER(0),
    SOURCE(0);

    POSITION(double deg) {
      degrees = deg;
    }

    double degrees = 0;
  }

  private void moveToPosition(POSITION pos) {
    if (m_PeriodicIO.controlState != CONTROL_STATE.MOTION_MAGIC) {
      m_PeriodicIO.controlState = CONTROL_STATE.MOTION_MAGIC;
    }
    m_PeriodicIO.targetPosition = pos.degrees;
    m_pivot.setControl(m_positionVoltageRequest.withPosition(m_PeriodicIO.targetPosition));
  }

  private boolean isAtTarget() {
    return m_pivot.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled
        && Math.abs(m_PeriodicIO.position - m_PeriodicIO.targetPosition) < 2;
  }

  public Command getGotoPositionCommand(POSITION pos) {
    return runOnce(() -> moveToPosition(pos));
  }

  public Command getGotoAndWaitPositionCommand(POSITION pos) {
    return getGotoPositionCommand(pos)
    .alongWith(Commands.none().until(() -> isAtTarget()));
  }
}
