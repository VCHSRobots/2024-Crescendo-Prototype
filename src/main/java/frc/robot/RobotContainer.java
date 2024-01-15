// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Pivot m_pivot = new Pivot();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // left y shooter speed
    m_shooter.setDefaultCommand(new RunCommand(() -> {
      double leftY = m_driverController.getLeftY();
      if (Math.abs(leftY) > 0.04) {
        m_shooter.shoot(m_driverController.getLeftY());
      } else {
        m_shooter.shoot(0);
      }
    }, m_shooter));

    m_intake.setDefaultCommand(Commands.run(()->m_intake.stop(), m_intake));

    // right bumper lower pivot
    m_driverController.rightBumper()
        .whileTrue(new RunCommand(() -> m_pivot.set(0.3), m_pivot).finallyDo(() -> m_pivot.set(0)));
    // right bumper raise pivot
    m_driverController.leftBumper()
        .whileTrue(new RunCommand(() -> m_pivot.set(-0.3), m_pivot).finallyDo(() -> m_pivot.set(0)));

    // right trigger intake speed
    m_driverController.rightTrigger(0.1)
        .whileTrue(new RunCommand(() -> m_intake.set(m_driverController.getRightTriggerAxis()), m_intake));
    // left trigger reverse speed
    m_driverController.leftTrigger(0.1)
        .whileTrue(new RunCommand(() -> m_intake.set(-m_driverController.getLeftTriggerAxis()), m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
