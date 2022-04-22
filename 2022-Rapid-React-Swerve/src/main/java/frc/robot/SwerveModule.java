package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;
  //private final PIDController turningPIDController;
  // The  motor encoder is part of the TalonFX
  private final CANCoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  private final PIDController turningPIDController;  
  
/*
 * Constructs a SwerveModule.
 *
 * @param driveMotorId ID for the drive motor.
 * @param turningMotorId ID for the turning motor.
 * @param absoluteEncoderId ID for the absolute turning encoder (CANCODER).
 */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
      
    absoluteEncoder = new CANCoder(absoluteEncoderId);
    driveMotor = new WPI_TalonFX(driveMotorId);
    turningMotor = new WPI_TalonFX(turningMotorId);
      
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    
    // This is the drive motor encoder
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    //driveMotor.setSensorPhase(true);
    // Turning motor encoder
    turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    //turningMotor.setSensorPhase(true);

    // Initialized using P gain only
    turningPIDController = new PIDController(Constants.kPTurning, 0, 0);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Need to write this
    // resetEncoders();
  }

  // Return drive position in meters
  // This is where the conversion must be done
  public double getDrivePosition()  {
    double distance = 
    (driveMotor.getSelectedSensorPosition() * Constants.kDriveEncoderRot2Meter)
    / Constants.kEncoderResolution;
    
    return (distance);
  }

  public double getTurningPosition()  {
    double angle = 
      (turningMotor.getSelectedSensorPosition()/Constants.kTurningMotorGearRatio)/Constants.kEncoderResolution * 2 * Math.PI;
    return angle;
    
  }

  public double getDriveVelocity()  {
    double velocity = 
    (driveMotor.getSelectedSensorVelocity() * 10 * Constants.kDriveMotorGearRatio * Math.PI * Constants.kWheelDiameterMeters) / 2048;
    
    return velocity;
  }

  public double getTurningVelocity()  {
    double velocity = 
      (turningMotor.getSelectedSensorVelocity() * 10 * Constants.kTurningMotorGearRatio * 2 * Math.PI) / 2048;
    
    return velocity;
  }

  public double getAbsoluteEncoderRad() {
    // Configure the Encoder to return position in -180 to +180 degrees, (0 to 360) by default
    double angle = absoluteEncoder.getAbsolutePosition() * (Math.PI/180);
    angle -= absoluteEncoderOffsetRad;
    
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveMotor.setSelectedSensorPosition(0.0);
    // What to do with the turning encoders?
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond/Constants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
  }

  public void stop()  {
    driveMotor.set(0);
    turningMotor.set(0);
  }

}
