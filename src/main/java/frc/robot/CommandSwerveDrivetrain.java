package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util.ModifiedSignalLogger;
import frc.robot.Util.SwerveVoltageRequest;
import frc.robot.Util.SysIdRoutine;
import frc.robot.Util.SysIdRoutine.Direction;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private Pose2d m_targetRobotPose = new Pose2d();

    private double m_targetRotation = 0;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        TunerConstants.maxModuleRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // don't flip path TODO verify
                                                                                         // how to change this correctly
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void periodic() {
    SmartDashboard.putNumber("current Rotation", getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("target Rotation", m_targetRobotPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Last tag Y rotation", Units.radiansToDegrees(lastTag.getRotation().getY()));
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    private SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
                    null,
                    this));

    private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

    private SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
                    null,
                    this));

    private SysIdRoutine m_slipSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
                    null,
                    this));

    public Command runDriveQuasiTest(Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction) {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    public Command runDriveSlipTest() {
        return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public void setCurrentLimitDrive(double currentLimit) {

        for (var module : Modules) {
            var drive = module.getDriveMotor();

            CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();

            StatusCode refresh = drive.getConfigurator().refresh(currentConfig, 0.1);

            if (!refresh.isOK()) {
                System.out.println(
                        "TalonFX ID " + drive.getDeviceID() + " failed refresh Current configs with error "
                                + refresh.toString());
            }

            currentConfig.SupplyCurrentLimit = currentLimit;
            currentConfig.SupplyCurrentThreshold = currentLimit;
            currentConfig.SupplyTimeThreshold = 0;
            currentConfig.SupplyCurrentLimitEnable = true;

            StatusCode response = drive.getConfigurator().apply(currentConfig);
            if (!response.isOK()) {
                System.out.println(
                        "TalonFX ID " + drive.getDeviceID() + " failed config with error " + response.toString());
            }
        }
    }

    public void setTorqueCurrentLimitDrive(double torqueCurrentLimit) {
        for (var module : Modules) {
            var drive = module.getDriveMotor();
            TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs();
            StatusCode refreshTorque = drive.getConfigurator().refresh(torqueCurrentConfigs, 0.1);

            if (!refreshTorque.isOK()) {
                System.out.println(
                        "TalonFX ID " + drive.getDeviceID() + " failed refresh Current configs with error "
                                + refreshTorque.toString());
            }
            torqueCurrentConfigs.PeakForwardTorqueCurrent = torqueCurrentLimit;
            torqueCurrentConfigs.PeakReverseTorqueCurrent = -torqueCurrentLimit;
        }
    }

    Pose3d lastTag = new Pose3d();

    public SwerveRequest limeLightTracking() {
        Pose2d currentPose = getState().Pose;
        double x = 0;
        double y = 0;
        double r = 0;
        double maxSpeed = 3.0;
        Pose2d goal = new Pose2d(new Translation2d(1.5, 0), new Rotation2d());

        if (RobotContainer.vision.isTargetValid()) {
            lastTag = RobotContainer.vision.getTagPoseRobotSpace();
            var dX = (lastTag.getZ() - goal.getX());
            var targetX = currentPose.getX()+dX;

            var dY = (lastTag.getX() - goal.getY());
            var targetY = currentPose.getY()+dY;

            var dR = (lastTag.getRotation().getY() - goal.getRotation().getRadians());
            var targetR = currentPose.getRotation().minus(Rotation2d.fromDegrees(dR));        

            SmartDashboard.putNumber("Delta Pose X", dX);
             SmartDashboard.putNumber("Delta Pose Y", dY); 
              SmartDashboard.putNumber("Delta Pose R", dR);
            SmartDashboard.putNumber("target R", targetR.getDegrees());

            m_targetRobotPose = new Pose2d(new Translation2d(targetX, targetY), targetR);
        }
        
        if (currentPose.getTranslation().getDistance(m_targetRobotPose.getTranslation()) > 0.1) {
            x = m_targetRobotPose.getX()-currentPose.getX();
            x = Math.copySign(Math.min(Math.abs(x), maxSpeed), x);

            y = m_targetRobotPose.getY()-currentPose.getY();
            y = -Math.copySign(Math.min(Math.abs(y), maxSpeed), y);

            r = m_targetRobotPose.getRotation().getDegrees()-currentPose.getRotation().getDegrees();
            r = Math.copySign(Math.min(Math.abs(r), 270), r);
            SmartDashboard.putNumber("r rate", r);
        }
        
       
        var forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        return forwardStraight.withDeadband(.05).withVelocityX(x).withVelocityY(y)
                .withRotationalRate(Units.degreesToRadians(r));
    }
}
