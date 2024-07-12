package frc.robot.util.vlib.control;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.vlib.TunableDouble;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

//this one is my idea though
public class VPIDController extends PIDController {
    TunableDouble kp;
    TunableDouble ki;
    TunableDouble kd;
    public VPIDController(String key, double kp, double ki, double kd) {
        super(kp, ki, kd);

        this.kp = new TunableDouble(key, "kp", kp);
        this.ki = new TunableDouble(key, "ki", ki);
        this.kd = new TunableDouble(key, "kd", kd);
    }

    @Override
    public double calculate(double measurement) {
        super.setP(kp.get());
        super.setI(ki.get());
        super.setD(kd.get());
        return super.calculate(measurement);
    }
}
