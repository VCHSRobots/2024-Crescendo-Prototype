// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class SRXPivot extends SubsystemBase {
  private final WPI_TalonSRX m_pivotMaster = new WPI_TalonSRX(PivotConstants.kPivotMasterId.getDeviceNumber());
  private final WPI_TalonSRX m_pivotLeader = new WPI_TalonSRX(PivotConstants.kPivotFollowerId.getDeviceNumber());

  /** Creates a new SRXPivot. */
  public SRXPivot() {

  }
}
