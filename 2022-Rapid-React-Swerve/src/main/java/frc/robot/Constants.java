package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    // Swerve module characteristics
    private static final double kWheelRadius = 0.0508;  // Supply real values when available (looks like 4"?)
    public static final int kEncoderResolution = 2048; // This looks right, but double-check
    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared - how was this derived?
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;     // for SDS
    public static final double kTurningMotorGearRatio = 1 / 12.8;   // for SDS
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;

    // Drive kinematics
    public static final double kTrackWidth = Units.inchesToMeters(21.0);
    public static final double kWheelBase = Units.inchesToMeters(21.0);
    
    // Individual swerve module parameters
    // Front left
    public static final int kFrontLeftDriveMotorPort = 0;
    public static final int kFrontLeftTurningMotorPort = 0;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

    // Front right
    public static final int kFrontRightDriveMotorPort = 0;
    public static final int kFrontRightTurningMotorPort = 0;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

    // Back left
    public static final int kBackLeftDriveMotorPort = 0;
    public static final int kBackLeftTurningMotorPort = 0;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

    // Back right
    public static final int kBackRightDriveMotorPort = 0;
    public static final int kBackRightTurningMotorPort = 0;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderPort = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

}
