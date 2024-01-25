// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_leftWinchMotor = new CANSparkMax(ClimberConstants.kLeftWinchMotorId, MotorType.kBrushless);
  private final CANSparkMax m_rightWinchMotor = new CANSparkMax(ClimberConstants.kRightWinchMotorId, MotorType.kBrushless);

  /** Creates a new Climber. */
  public Climber() {
    m_leftWinchMotor.restoreFactoryDefaults();
    m_rightWinchMotor.restoreFactoryDefaults();

    m_leftWinchMotor.setIdleMode(IdleMode.kBrake);
    m_rightWinchMotor.setIdleMode(IdleMode.kBrake);
    m_leftWinchMotor.setInverted(true);
    m_rightWinchMotor.setInverted(false);

    m_leftWinchMotor.setSmartCurrentLimit(40, 40);
    m_rightWinchMotor.setSmartCurrentLimit(40, 40);
  }

  public void setLeftWinch(double percentOut) {
    m_leftWinchMotor.set(percentOut);
  }

  public void setRightWinch(double percentOut) {
    m_rightWinchMotor.set(percentOut);
  }
}
