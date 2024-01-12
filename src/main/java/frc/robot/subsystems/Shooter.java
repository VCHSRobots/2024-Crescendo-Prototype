// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX m_shooterMasterMotor = new TalonFX(ShooterConstants.kShooterMasterMotorId);
  private final TalonFX m_shooterFollowerMotor = new TalonFX(ShooterConstants.kShooterFollowerMotorId);

  private VoltageOut m_shooterVoltageOut = new VoltageOut(0);

  /** Creates a new Intake. */
  public Shooter() {
    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMotorConfig.CurrentLimits.SupplyCurrentThreshold = 30;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    shooterMotorConfig.CurrentLimits.SupplyTimeThreshold = .00;

    m_shooterMasterMotor.getConfigurator().apply(shooterMotorConfig);
    m_shooterFollowerMotor.getConfigurator().apply(shooterMotorConfig);

    m_shooterFollowerMotor.setControl(new Follower(m_shooterMasterMotor.getDeviceID(), false));
  }

  public Command getShootCommand() {
    return new RunCommand(() -> shoot(0.6), this).finallyDo(this::stop);
  }

  public void shoot(double percentOut) {
    m_shooterMasterMotor.setControl(m_shooterVoltageOut.withOutput(Math.max(12 * percentOut, 12)));
  }

  public void stop() {
    m_shooterMasterMotor.setControl(m_shooterVoltageOut.withOutput(0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("velocity output (Rot/s): ", () -> m_shooterMasterMotor.getVelocity().getValueAsDouble(), null);
      builder.addDoubleProperty("supply current: ", () -> m_shooterMasterMotor.getSupplyCurrent().getValueAsDouble(), null);
      builder.addDoubleProperty("stator current: ", () -> m_shooterMasterMotor.getStatorCurrent().getValueAsDouble(), null);
      builder.addDoubleProperty("voltage: ", () -> m_shooterMasterMotor.getMotorVoltage().getValueAsDouble(), null);
  }
}
