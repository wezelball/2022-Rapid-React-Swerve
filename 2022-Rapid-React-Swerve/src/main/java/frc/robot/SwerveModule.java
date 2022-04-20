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
import edu.wpi.first.wpilibj.controller.PIDController;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class SwerveModule {
    
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;
  private final PIDController turningPIDController;
  // The  motor encoder is part of the TalonFX
  private final CANCoder turningEncoder;
  private final boolean turningEncoderReversed;
  private final double turningEncoderOffsetRad;
  
  
  // The driving PID controller with P gain only (kP, kI, kD)
  //private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  /*
  This is the turning PID controller, the constructor looks like this:
  ProfiledPIDController​(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints)
  The TrapezoidProfile Constraints create a profile that is constrained by the modules 
  maximum angular velocity and acceleration
  */
  //private final ProfiledPIDController m_turningPIDController =
  //  new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  /*
  Feedforward uses the system gains for modeling the motor: constructors are as follows:
  SimpleMotorFeedforward​(double ks, double kv)
  SimpleMotorFeedforward​(double ks, double kv, double ka)
  
  Where,
  ks - The static gain.
  kv - The velocity gain.
  ka - The acceleration gain.
  
  Note that this is designed for a somple permanent-magnet motor, so we need to 
  determine if this applies to brushless motors like the Falcon.
  */
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
 * Constructs a SwerveModule.
 *
 * @param driveMotorId ID for the drive motor.
 * @param turningMotorId ID for the turning motor.
 */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    
    this.turningEncoderOffsetRad = absoluteEncoderOffset;
    this.turningEncoderReversed = absoluteEncoderReversed;
      
    turningEncoder = new CANCoder;
    driveMotor = new WPI_TalonFX(driveMotorId);
    turningMotor = new WPI_TalonFX(turningMotorId);
      
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    //m_driveMotor = new PWMSparkMax(driveMotorChannel);
    //m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    //m_turningMotor = new PWMSparkMax(turningMotorChannel);
    //m_turningMotor = new WPI_TalonFX(turningMotorChannel);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);    //FIXME

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);                 //FIXME

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

    
}
