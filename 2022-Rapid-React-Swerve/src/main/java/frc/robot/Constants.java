package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    private static final double kWheelRadius = 0.0508;  // Supply real values when available (looks like 4"?)
    public static final int kEncoderResolution = 2048; // This looks right, but double-check

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared - how was this derived?
 
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;     // for SDS
    public static final double kTurningMotorGearRatio = 1 / 12.8;   // for SDS
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;

}
