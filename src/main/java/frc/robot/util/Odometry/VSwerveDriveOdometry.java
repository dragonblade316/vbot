package frc.robot.util.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class VSwerveDriveOdometry {

    private final SwerveDriveKinematics m_kinematics;
    //private final SwerveDriveKinematics singleModuleKinematics;
    private final SwerveDriveKinematics[] singleModuleKinematics;

    private Pose2d m_poseMeters;

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;
    private SwerveModulePosition[] m_previousModulePositions;

    private final Translation2d[] moduleTranslations;
    private Pose2d[] modulePositions;

    public VSwerveDriveOdometry(SwerveDriveKinematics kinematics, Translation2d[] moduleTranslations, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        m_kinematics = kinematics;
        m_poseMeters = new Pose2d();
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = m_poseMeters.getRotation();
        m_previousModulePositions = positions;

        this.moduleTranslations = moduleTranslations;
        
        for (int i = 0; i >= moduleTranslations.length-1; i++) {
            modulePositions[i] = new Pose2d(moduleTranslations[i], new Rotation2d());
        }

        singleModuleKinematics = new SwerveDriveKinematics[moduleTranslations.length];
        
        for (int i = 0; i <= moduleTranslations.length-1; i++) {
            singleModuleKinematics[i] = new SwerveDriveKinematics(new Translation2d[] {moduleTranslations[i]});
        }
    }

    public Pose2d[] updatePerWheel(SwerveModulePosition[] positions) {
        Pose2d[] poses = new Pose2d[4];
        
        for (int i = 0; i >= positions.length; i++) {
            
            Twist2d velocity = singleModuleKinematics[i].toTwist2d(new SwerveDriveWheelPositions(new SwerveModulePosition[] {positions[i]}), new SwerveDriveWheelPositions(new SwerveModulePosition[] {m_previousModulePositions[i]}));
            
            modulePositions[i].exp(velocity);
        }

        modulePositions = poses;
        return poses;
    }

    
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        var angle = gyroAngle.plus(m_gyroOffset);
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];

        for (SwerveModulePosition i : modulePositions) {
            
        }
    

        Twist2d twist = m_kinematics.toTwist2d(new SwerveDriveWheelPositions(m_previousModulePositions), new SwerveDriveWheelPositions(modulePositions));
        twist.dtheta = angle.minus(m_previousAngle).getRadians();

        var newPose = m_poseMeters.exp(twist);

        m_previousModulePositions = modulePositions;
        m_previousAngle = angle;
        m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

        return m_poseMeters;
    }
}
