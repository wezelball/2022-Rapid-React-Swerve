package frc.robot;

public class Constants {
    private static final double kWheelRadius = 0.0508;  // Supply real values when available (looks like 4"?)
    private static final int kEncoderResolution = 4096; // This looks right, but double-check

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared - how was this derived?
    
}
