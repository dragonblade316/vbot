package frc.robot.util.vlib.control;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.vlib.TunableDouble;

public class VSimpleMotorFeedforward extends SimpleMotorFeedforward {
    private TunableDouble ks;
    private TunableDouble kv;


    public VSimpleMotorFeedforward(String id) {
        super(1, 1);
    }


}
