// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX m_shooterMasterMotor = new TalonFX(ShooterConstants.kShooterTopMasterMotorId.getDeviceNumber(),
      ShooterConstants.kShooterTopMasterMotorId.getBus());
  private final TalonFX m_shooterFollowerMotor = new TalonFX(
      ShooterConstants.kShooterBottomFollowerMotorId.getDeviceNumber(),
      ShooterConstants.kShooterBottomFollowerMotorId.getBus());

  private final VoltageOut m_shooterVoltageOut = new VoltageOut(0);
  private final MotionMagicVelocityVoltage m_shooterVelocityMotionMagic = new MotionMagicVelocityVoltage(0);
  private final StatusSignal<Double> m_shooterVelocity = m_shooterMasterMotor.getVelocity();
  private double motionMagicTargetVelocity = 0;

  private double currVoltage = 0;

  /** Creates a new Shooter. */
  public Shooter() {
    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    shooterMotorConfig.CurrentLimits.SupplyTimeThreshold = .00;

    shooterMotorConfig.Feedback.SensorToMechanismRatio = 1;
    shooterMotorConfig.Feedback.RotorToSensorRatio = 1;

    // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html#using-motion-magic-velocity-in-api
    var slot0Configs = shooterMotorConfig.Slot0; // TODO shoot pid ksva
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    var motionMagicConfigs = shooterMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 50;
    motionMagicConfigs.MotionMagicJerk = 0;

    m_shooterMasterMotor.getConfigurator().apply(shooterMotorConfig);
    m_shooterFollowerMotor.getConfigurator().apply(shooterMotorConfig);

    m_shooterFollowerMotor.setControl(new Follower(m_shooterMasterMotor.getDeviceID(), false)); // oppose = true
  }

  /*
   * NORMAL FUNCTIONALITY
   */

  public void shoot(double percentOut) {
    double maxVoltage = 12.0;
    double voltage = MathUtil.clamp(maxVoltage * percentOut, -maxVoltage, maxVoltage);
    m_shooterMasterMotor.setControl(m_shooterVoltageOut.withOutput(voltage));
  }

  public void stop() {
    m_shooterMasterMotor.setControl(m_shooterVoltageOut.withOutput(0));
  }

  public boolean isMotionMagicVelocityAtTarget() {
    return Math.abs(m_shooterVelocity.getValueAsDouble() - motionMagicTargetVelocity) < 10;
  }

  public void setTargetVelocity(double velocity) {
    motionMagicTargetVelocity = velocity;
    m_shooterMasterMotor.setControl(m_shooterVelocityMotionMagic.withVelocity(velocity));
  }

  public double getTargetVelocity() {
    return motionMagicTargetVelocity;
  }

  public void increaseVoltage(double inc) {
    currVoltage = MathUtil.clamp(currVoltage + inc, 0, 12);
  }

  public void decreaseVoltage(double dec) {
    currVoltage = MathUtil.clamp(currVoltage - dec, 0, 12);
  }

  public double getStoredVoltage() {
    return currVoltage;
  }

  public void setToCurrVoltage() {
    m_shooterMasterMotor.setControl(m_shooterVoltageOut.withOutput(currVoltage));
  }

  @Override
  public void periodic() {
    m_shooterVelocity.refresh(); // refresh signal to get more recent values
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    setName("Shooter");
    super.initSendable(builder);
    builder.addDoubleProperty("velocity output (Rot/s): ", () -> m_shooterVelocity.getValueAsDouble(),
        null);
    builder.addDoubleProperty("supply current: ", () -> m_shooterMasterMotor.getSupplyCurrent().getValueAsDouble(),
        null);
    builder.addDoubleProperty("stator current: ", () -> m_shooterMasterMotor.getStatorCurrent().getValueAsDouble(),
        null);
    builder.addDoubleProperty("voltage: ", () -> m_shooterMasterMotor.getMotorVoltage().getValueAsDouble(), null);
    builder.addDoubleProperty("target velocity: ", () -> getTargetVelocity(), null);
    builder.addDoubleProperty("curr stored voltage ", () -> getStoredVoltage(), null);
  }

  /*
   * COMMAND FACTORIES
   */
  public Command getSpeedUpUntilVelocityCommand(double velocity) {
    return Commands.sequence(
        getSpeedUpVelocityCommand(velocity),
        Commands.idle(this).until(this::isMotionMagicVelocityAtTarget));
  }

  public Command getSpeedUpVelocityCommand(double velocity) {
    return runOnce(() -> setTargetVelocity(velocity));
  }

  public Command getShootCommand() {
    return new RunCommand(() -> shoot(0.6), this).finallyDo(this::stop);
  }

  public Command getShootCloseCommand() {
    return new RunCommand(() -> shoot(0.5), this).finallyDo(this::stop);
  }

  public Command getShootMidCommand() {
    return new RunCommand(() -> shoot(0.7), this).finallyDo(this::stop);
  }
}
