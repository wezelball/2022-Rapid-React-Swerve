package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    
    /*
     This describes the location of the swerve modules, relative to the center of the
     robot.  Positive x values represent moving toward the front of the robot whereas 
     positive y values represent moving toward the left of the robot. Units are in
     meters.
     */
    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);      // Supply real values when available
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);    // Supply real values when available
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);      // Supply real values when available
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);    // Supply real values when available
    
}
