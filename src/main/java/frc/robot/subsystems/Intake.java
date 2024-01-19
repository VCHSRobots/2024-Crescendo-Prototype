// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final TalonFX m_intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId.getDeviceNumber(),
      IntakeConstants.kIntakeMotorId.getBus());
  private final DigitalInput m_proxSensor = new DigitalInput(IntakeConstants.kProxSensorChannel);

  private VoltageOut m_intakeVoltageOut = new VoltageOut(0);

  /** Creates a new Intake. */
  public Intake() {
    TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.SupplyCurrentThreshold = 20;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    intakeMotorConfig.CurrentLimits.SupplyTimeThreshold = .00;

    m_intakeMotor.getConfigurator().apply(intakeMotorConfig);
  }

  /*
   * NORMAL FUNCTIONALITY
   */
  public boolean isPieceAtIntake() {
    return !m_proxSensor.get();
  }

  public void intake() {
    m_intakeMotor.setControl(m_intakeVoltageOut.withOutput(2));
  }

  public void feedShoot() {
    m_intakeMotor.setControl(m_intakeVoltageOut.withOutput(8));
  }

  public void reverse() {
    m_intakeMotor.setControl(m_intakeVoltageOut.withOutput(-2));
  }

  public void stop() {
    m_intakeMotor.setControl(m_intakeVoltageOut.withOutput(0));
  }

  public void set(double percentOut) {
    m_intakeMotor.set(percentOut);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("velocity output (Rot/s): ", () -> m_intakeMotor.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("supply current: ", () -> m_intakeMotor.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("stator current: ", () -> m_intakeMotor.getStatorCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("voltage: ", () -> m_intakeMotor.getMotorVoltage().getValueAsDouble(), null);
    builder.addBooleanProperty("piece at intake", () -> isPieceAtIntake(), null);
  }

  /*
   * COMMANDS
   */
  public Command getIntakeCommand() {
    return run(this::intake).finallyDo(this::stop);
  }

  public Command getReverseCommand() {
    return run(this::reverse).finallyDo(this::stop);
  }

  public Command getFeedShooterCommand() {
    return run(this::feedShoot).finallyDo(this::stop);
  }

  public Command getStopCommand() {
    return runOnce(this::stop);
  }

  public Command getIntakeUntilPieceCommand() {
    return new ConditionalCommand(
        run(this::intake).until(() -> isPieceAtIntake()),
        runOnce(this::stop),
        () -> !isPieceAtIntake());
  }
}
