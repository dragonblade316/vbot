package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public class RobotState {
    private static RobotState instance;
    private RobotState() {}

    public static RobotState get_instance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    //drive base
    public Pose2d robotPose = new Pose2d();
    public Twist2d translationVelocity = new Twist2d();

    //will only ever be used with the heading controller
    public boolean aligned = false;

    //intake and carrier:
    public boolean containsPiece = false;
    public boolean barfing = false;

    //shooter:

    //might move this to the shooter subsystem
    public enum FlywheelState {
        READY,
        ACCELERATING,
        INACTIVE
    }

    public FlywheelState shooterState = FlywheelState.INACTIVE;
}
