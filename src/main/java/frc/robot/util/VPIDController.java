package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class VPIDController extends PIDController {
    LoggedDashboardNumber kp;
    LoggedDashboardNumber ki;
    LoggedDashboardNumber kd;
    public VPIDController(String key, double kp, double ki, double kd) {
        super(kp, ki, kd);

        this.kp = new LoggedDashboardNumber(key + "/kp");
        this.kp.set(kp);
        this.ki = new LoggedDashboardNumber(key + "/ki");
        this.ki.set(kp);
        this.kd = new LoggedDashboardNumber(key + "/kd");
        this.kd.set(kp);
    }

    @Override
    public double calculate(double measurement) {

        super.setP(kp.get());
        super.setI(ki.get());
        super.setD(kd.get());

        return super.calculate(measurement);
    }
}
