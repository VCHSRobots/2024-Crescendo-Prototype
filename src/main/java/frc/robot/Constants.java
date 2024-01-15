// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.team4415.drivers.CanDeviceId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    public static final CanDeviceId kIntakeMotorId = new CanDeviceId(1, kRioBus);
    public static final int kProxSensorChannel = 0;
  }

  public static class ShooterConstants {
    public static final CanDeviceId kShooterTopMasterMotorId = new CanDeviceId(3, kRioBus);
    public static final CanDeviceId kShooterBottomFollowerMotorId = new CanDeviceId(4, kRioBus);
  }

  public static class PivotConstants {
    public static CanDeviceId kPivotMotor = new CanDeviceId(5, kRioBus);
  }

  public static String kCanivoreBus = "canivore";
  public static String kRioBus = "rio";
}
