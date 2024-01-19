package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
                        .withKP(150).withKI(0).withKD(20)
                        .withKS(0.27).withKV(1.5).withKA(0.027);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
                        .withKP(3).withKI(0).withKD(0)
                        .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 120.0;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final double kWheelRadiusInches = 2;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 107.0 / kDriveGearRatio * Math.PI * 2
                        * Units.inchesToMeters(kWheelRadiusInches); // 107 rps =

        private static final boolean kSteerMotorReversed = true;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "Default Name";
        private static final int kPigeonId = 10;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                        .withPigeon2Id(kPigeonId)
                        .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches)
                        .withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 11;
        private static final int kFrontLeftSteerMotorId = 12;
        private static final int kFrontLeftEncoderId = 19;
        private static final double kFrontLeftEncoderOffset = -0.191895; // -0.191162; //-0.191650; //-0.19091796875; //
                                                                         // 0.191650

        private static final double kFrontLeftXPosInches = 10.375;
        private static final double kFrontLeftYPosInches = 10.375;

        // Front Right
        private static final int kFrontRightDriveMotorId = 17;
        private static final int kFrontRightSteerMotorId = 18;
        private static final int kFrontRightEncoderId = 22;
        private static final double kFrontRightEncoderOffset = -0.341309;// -0.361328;
                                                                         // //-0.340576;//-0.347656;//-0.35888671875;

        private static final double kFrontRightXPosInches = 10.375;
        private static final double kFrontRightYPosInches = -10.375;

        // Back Left
        private static final int kBackLeftDriveMotorId = 13;
        private static final int kBackLeftSteerMotorId = 14;
        private static final int kBackLeftEncoderId = 20;
        private static final double kBackLeftEncoderOffset = -0.370117;// -0.370605; //-0.371094; //-0.37109375; //
                                                                       // 0.371094

        private static final double kBackLeftXPosInches = -10.375;
        private static final double kBackLeftYPosInches = 10.375;

        // Back Right
        private static final int kBackRightDriveMotorId = 15;
        private static final int kBackRightSteerMotorId = 16;
        private static final int kBackRightEncoderId = 21;
        private static final double kBackRightEncoderOffset = 0.221680;// 0.222412;//
                                                                       // 0.212646;//0.188232;//0.08447265625;

        private static final double kBackRightXPosInches = -10.375;
        private static final double kBackRightYPosInches = -10.375;

        private static final double FrontLeftPos = Math
                        .sqrt(Math.pow(kFrontLeftXPosInches, 2) + Math.pow(kFrontLeftYPosInches, 2));
        private static final double FrontRightPos = Math
                        .sqrt(Math.pow(kFrontRightXPosInches, 2) + Math.pow(kFrontRightYPosInches, 2));
        private static final double BackLeftPos = Math
                        .sqrt(Math.pow(kBackLeftXPosInches, 2) + Math.pow(kBackLeftYPosInches, 2));
        private static final double BackRightPos = Math
                        .sqrt(Math.pow(kBackRightXPosInches, 2) + Math.pow(kBackRightYPosInches, 2));
        public static final double maxModuleRadius = Math.max(FrontLeftPos,
                        Math.max(FrontRightPos, Math.max(BackRightPos, BackLeftPos)));

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                        Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
                        kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                        kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                        Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
                        kInvertLeftSide);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                        Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                        kInvertRightSide);

        public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,
                        FrontLeft,
                        FrontRight, BackLeft, BackRight);

}
