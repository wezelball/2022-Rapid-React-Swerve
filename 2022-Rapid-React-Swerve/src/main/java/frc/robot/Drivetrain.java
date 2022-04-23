package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.kFrontLeftDriveMotorPort,
        Constants.kFrontLeftTurningMotorPort,
        Constants.kFrontLeftDriveEncoderReversed,
        Constants.kFrontLeftTurningEncoderReversed,
        Constants.kFrontLeftDriveAbsoluteEncoderPort,
        Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        Constants.kFrontLeftDriveAbsoluteEncoderReversed);

        private final SwerveModule frontRight = new SwerveModule(
        Constants.kFrontRightDriveMotorPort,
        Constants.kFrontRightTurningMotorPort,
        Constants.kFrontRightDriveEncoderReversed,
        Constants.kFrontRightTurningEncoderReversed,
        Constants.kFrontRightDriveAbsoluteEncoderPort,
        Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        Constants.kFrontRightDriveAbsoluteEncoderReversed);
    
        private final SwerveModule backLeft = new SwerveModule(
        Constants.kBackLeftDriveMotorPort,
        Constants.kBackLeftTurningMotorPort,
        Constants.kBackLeftDriveEncoderReversed,
        Constants.kBackLeftTurningEncoderReversed,
        Constants.kBackLeftDriveAbsoluteEncoderPort,
        Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        Constants.kBackLeftDriveAbsoluteEncoderReversed);

        private final SwerveModule backRight = new SwerveModule(
        Constants.kBackRightDriveMotorPort,
        Constants.kBackRightTurningMotorPort,
        Constants.kBackRightDriveEncoderReversed,
        Constants.kBackRightTurningEncoderReversed,
        Constants.kBackRightDriveAbsoluteEncoderPort,
        Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
        Constants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0));
    
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.kWheelBase/2, -Constants.kTrackWidth/2),
        new Translation2d(Constants.kWheelBase/2, Constants.kTrackWidth/2),
        new Translation2d(-Constants.kWheelBase/2, -Constants.kTrackWidth/2),
        new Translation2d(-Constants.kWheelBase/2, Constants.kTrackWidth/2)
    );

    public void zeroHeading()   {
        gyro.reset();
    }
    
    public double getHeading()  {
        // Do we actually need this below?
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    
    public Rotation2d getRotation2d()   {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose)  {
        odometer.resetPosition(pose, getRotation2d());
    }

    public void update() {
        odometer.update
            (getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot location", getPose().getTranslation().toString());
    }

    public void stopModules()   {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /*
    public void setModuleStates(SwerveModuleState[] desiredStates)  {
        //SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        // They changed the name from normaize to desaturate - really???
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
    }
    */

    // This is from wpilib swervebot, and needs some tweaking to work
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Need to translate this section
        var swerveModuleStates =
            kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        
        // We are already doing this in setModuleStates
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
      }
}
