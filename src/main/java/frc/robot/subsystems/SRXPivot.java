// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class SRXPivot extends SubsystemBase {
  private final WPI_TalonSRX m_pivotMaster = new WPI_TalonSRX(PivotConstants.kPivotMasterId.getDeviceNumber());
  private final WPI_TalonSRX m_pivotFollower = new WPI_TalonSRX(PivotConstants.kPivotFollowerId.getDeviceNumber());

  private final double m_gearRatio = 1;

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
    pivotConfig.slot0.kP = 0;
    pivotConfig.slot0.kI = 0;
    pivotConfig.slot0.kD = 0;
    pivotConfig.slot0.kF = 0;

    pivotConfig.motionCruiseVelocity = 0;
    pivotConfig.motionAcceleration = 0;
    pivotConfig.motionCurveStrength = 0;
  }

  public void goToPosition(double angle) {
    m_pivotMaster.set(ControlMode.MotionMagic, angleToTicks(angle));
  }

  public double angleToTicks(double angle) {
    return angle / 360.0 / m_gearRatio * 2048.0;
  }

  public double ticksToAngle(double ticks) {
    return ticks / 2048.0 * m_gearRatio * 360.0;
  }

  public void set(double percentOuput) {
    m_pivotMaster.set(percentOuput);
  }

}
