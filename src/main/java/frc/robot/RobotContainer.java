// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.SRXPivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SRXPivot.POSITION;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.SysIdRoutine.Direction;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;

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
  private final SRXPivot m_pivot = new SRXPivot();

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Double> speedChooser = new SendableChooser<>();
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // Initial max is true top speed
  private final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
  private final double MaxAngularRate = Math.PI * 2.0; // .75 rotation per second max angular velocity. Adjust for max
                                                       // turning rate speed.
  private final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity. Adjust for
                                                          // max turning rate speed.
  private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(1); // operator xbox controller
  private final CommandXboxController m_testController = new CommandXboxController(2);
  private final CommandXboxController m_sysidController = new CommandXboxController(3);

  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // drivetrain

  // Field-centric driving in Open Loop, can change to closed loop after
  // characterization
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(MaxSpeed * 0.01)
      .withRotationalDeadband(AngularRate * 0.01);
  // Field-centric driving in Closed Loop. Comment above and uncomment below.
  // SwerveRequest.FieldCentric drive = new
  // SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity).withDecoadband(MaxSpeed
  // * 0.1).withRotationalDeadband(AngularRate * 0.1);

  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  Limelight vision = new Limelight(drivetrain);

  Telemetry logger = new Telemetry(MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  private Supplier<SwerveRequest> controlStyle = () -> {
    return drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward -Y
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-m_driverController.getRightX() * AngularRate); // Drive counterclockwise with negative X
                                                                            // (left);
  };
  private Double lastSpeed = 0.65;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("intakePiece", m_intake.getIntakeUntilPieceCommand());
    NamedCommands.registerCommand("shootClose",
        Commands.sequence(m_shooter.getShootCloseCommand().withTimeout(1),
            m_intake.getFeedShooterCommand().withTimeout(1)));
    NamedCommands.registerCommand("shootMid",
        Commands.sequence(m_shooter.getShootMidCommand(), Commands.waitSeconds(1), m_intake.getFeedShooterCommand()));
    NamedCommands.registerCommand("pivotIntake", m_pivot.getGotoPositionUntilTargetCommand(POSITION.HOME));
    NamedCommands.registerCommand("pivotShootClose",
        m_pivot.getGotoPositionUntilTargetCommand(POSITION.SPEAKER_CLOSE_AUTO));
    NamedCommands.registerCommand("pivotShootMid",
        m_pivot.getGotoPositionUntilTargetCommand(POSITION.SPEAKER_MID_AUTO));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    speedChooser.addOption("100%", 1.0);
    speedChooser.addOption("95%", 0.95);
    speedChooser.addOption("90%", 0.9);
    speedChooser.addOption("85%", 0.85);
    speedChooser.addOption("80%", 0.8);
    speedChooser.addOption("75%", 0.75);
    speedChooser.addOption("70%", 0.7);
    speedChooser.setDefaultOption("65%", 0.65);
    speedChooser.addOption("60%", 0.6);
    speedChooser.addOption("55%", 0.55);
    speedChooser.addOption("50%", 0.5);
    speedChooser.addOption("35%", 0.35);
    SmartDashboard.putData("Speed Limit", speedChooser);

    // Configure the trigger bindings
    configureBindings();
    Shuffleboard.getTab("ss").add("intake", m_intake);
    Shuffleboard.getTab("ss").add("shooter", m_shooter);
    Shuffleboard.getTab("ss").add("pivot", m_pivot);
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
    // m_shooter.setDefaultCommand(new RunCommand(() -> {
    // double leftY = m_driverController.getLeftY();
    // if (Math.abs(leftY) > 0.04) {
    // m_shooter.shoot(m_driverController.getLeftY());
    // } else {
    // m_shooter.shoot(0);
    // }
    // }, m_shooter));

    m_intake.setDefaultCommand(Commands.run(() -> m_intake.stop(), m_intake));
    // m_pivot.setDefaultCommand(m_pivot.getHoldPositionCommand());

    // right bumper lower pivot
    m_driverController.rightBumper()
        .whileTrue(new RunCommand(() -> m_pivot.set(0.3), m_pivot)
            .finallyDo(() -> m_pivot.setTargetDegreesToCurrentPosition()));
    // right bumper raise pivot
    m_driverController.leftBumper()
        .whileTrue(new RunCommand(() -> m_pivot.set(-0.3), m_pivot)
            .finallyDo(() -> m_pivot.setTargetDegreesToCurrentPosition()));

    // right trigger intake speed
    m_driverController.rightTrigger(0.1)
        .whileTrue(new RunCommand(() -> m_intake.set(m_driverController.getRightTriggerAxis() * 0.75), m_intake));
    // left trigger reverse speed
    m_driverController.leftTrigger(0.1)
        .whileTrue(m_intake.getIntakeUntilPieceCommand());

    m_driverController.a().whileTrue(m_pivot.getGotoPositionCommand(POSITION.HOME));
    m_driverController.b().whileTrue(m_pivot.getGotoPositionCommand(POSITION.SPEAKER));
    m_driverController.x().whileTrue(m_pivot.getGotoPositionCommand(POSITION.AMP));
    // m_driverController.y().whileTrue(m_pivot.getGotoPositionCommand(POSITION.SOURCE));

    // increase shooter speed
    m_driverController.pov(0).onTrue(new InstantCommand(() -> m_shooter.increaseVoltage(.5)));
    // increase shooter speed
    m_driverController.pov(180).onTrue(new InstantCommand(() -> m_shooter.decreaseVoltage(.5)));
    // on/off shooter
    m_driverController.pov(90)
        .toggleOnTrue(new RunCommand(() -> m_shooter.setToCurrVoltage(), m_shooter).finallyDo(() -> m_shooter.stop()));

    // zero
    m_driverController.back().onTrue(new InstantCommand(() -> m_pivot.zero(), m_pivot));
    newSpeed();

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(controlStyle).ignoringDisable(true));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    m_driverController.pov(270).whileTrue(drivetrain.applyRequest(() -> {
      double x = 0;
      double y = 0;
      double r = 0;
      double maxSpeed = 2.0;
      var tag = vision.getTagPoseRobotSpace();
      Pose2d goal = new Pose2d(new Translation2d(1.5, 0), new Rotation2d()); // TODO check whether 180

      x = (tag.getZ() - goal.getX()) * 0.5;
      x = Math.copySign(Math.min(Math.abs(x), maxSpeed), x);

      y = (tag.getX() - goal.getY()) * 0.5;
      y = -Math.copySign(Math.min(Math.abs(y), maxSpeed), y);

      r = (Units.radiansToDegrees(tag.getRotation().getY()) - goal.getRotation().getDegrees());
      r = -Math.copySign(Math.min(Math.abs(r), 270), r);

      return forwardStraight.withDeadband(.05).withVelocityX(x).withVelocityY(y)
          .withRotationalRate(Units.degreesToRadians(r));
    }));

    // testing controller
    m_testController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_testController.b().whileTrue(drivetrain
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-m_testController.getLeftY(), -m_testController.getLeftX()))));
    m_testController.x().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d())));
    m_testController.y()
        .whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(90))));

        m_sysidController.x().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
        m_sysidController.x().and(m_sysidController.pov(180)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));
    
        m_sysidController.y().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
        m_sysidController.y().and(m_sysidController.pov(180)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));
    
        m_sysidController.a().and(m_sysidController.pov(0)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
        m_sysidController.a().and(m_sysidController.pov(180)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));
    
        m_sysidController.b().and(m_sysidController.pov(0)).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
        m_sysidController.b().and(m_sysidController.pov(180)).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));
    
        // Drivetrain needs to be placed against a sturdy wall and test stopped immediately upon wheel slip
        m_sysidController.back().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveSlipTest());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();

  }

  private void newSpeed() {
    lastSpeed = speedChooser.getSelected();
    MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * lastSpeed;
  }

  public void setPivotTargetToCurrentPosition() {
    m_pivot.setTargetDegreesToCurrentPosition();
  }

  public void setPivotStop() {
    m_pivot.stop();
  }
}
