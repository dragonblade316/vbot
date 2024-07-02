package frc.robot.util.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class VSwerveDriveOdometry {

    // private final SwerveDriveKinematics m_kinematics;
    // private final SwerveDriveKinematics singleModuleKinematics;
    // private Pose2d m_poseMeters;

    // private Rotation2d m_gyroOffset;
    // private Rotation2d m_previousAngle;
    // private SwerveModulePosition[] m_previousModulePositions;

    // private final Pose2d[] ModulePositions;

    // public VSwerveDriveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
    //     m_kinematics = kinematics;
    //     m_poseMeters = new Pose2d();
    //     m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    //     m_previousAngle = m_poseMeters.getRotation();
    //     m_previousModulePositions = positions;

    //     singleModuleKinematics = new SwerveDriveKinematics(new Translation2d[] {new Translation2d(0,0)});

    //     m_kinematics.
    // }



    
    // public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    //     var angle = gyroAngle.plus(m_gyroOffset);
    //     SwerveModulePosition[] deltas = new SwerveModulePosition[4];

    //     for (SwerveModulePosition i : modulePositions) {
            
    //     }
    

    //     Twist2d twist = m_kinematics.toTwist2d(new SwerveDriveWheelPositions(m_previousModulePositions), new SwerveDriveWheelPositions(modulePositions));
    //     twist.dtheta = angle.minus(m_previousAngle).getRadians();

    //     var newPose = m_poseMeters.exp(twist);

    //     m_previousModulePositions = modulePositions;
    //     m_previousAngle = angle;
    //     m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

    //     return m_poseMeters;
    //}
}
