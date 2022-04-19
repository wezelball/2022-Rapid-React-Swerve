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
    private static final double kWheelRadius = 0.0508;  // Supply real values when available (looks like 4"?)
    private static final int kEncoderResolution = 4096; // This looks right, but double-check

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared - how was this derived?

    // Used to be speedController, but that was deprecated, now MotorController
    private final MotorController m_driveMotor;
    private final MotorController m_turningMotor;

    // These are digital input quadrature encoders - not for us
    //private final Encoder m_driveEncoder = new Encoder(0, 1);
    //private final Encoder m_turningEncoder = new Encoder(2, 3);

    // The driving PID controller with P gain only (kP, kI, kD)
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    /*
    This is the turning PID controller, the constructor looks like this:
    ProfiledPIDController​(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints)
    The TrapezoidProfile Constraints create a profile that is constrained by the modules 
    maximum angular velocity and acceleration
    */
    private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

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
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
        //m_driveMotor = new PWMSparkMax(driveMotorChannel);
        m_driveMotor = new WPI_TalonFX(driveMotorChannel);
        //m_turningMotor = new PWMSparkMax(turningMotorChannel);
        m_turningMotor = new WPI_TalonFX(turningMotorChannel);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);    //FIXME

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.
        m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);                 //FIXME

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

    
}
