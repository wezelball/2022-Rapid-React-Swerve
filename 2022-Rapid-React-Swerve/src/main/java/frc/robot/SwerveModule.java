package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.MotorController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class SwerveModule {
    private static final double kWheelRadius = 0.0508;  // Supply real values when available (looks like 4"?)
    private static final int kEncoderResolution = 4096; // This looks right, but double-check

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared - how was this derived?

    // Used to be speedController, but that was deprecated
    private final MotorController m_driveMotor;
    private final MotorController m_turningMotor;
    
}
