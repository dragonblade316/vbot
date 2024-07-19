// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LedBuilder.Animations;

public class Leds extends SubsystemBase {
  private AddressableLED m_led;
  private final LedBuilder ledstate;
  private static final int LED_LENGTH = 82;

  /** Creates a new Leds. */
  public Leds() {
    ledstate = new LedBuilder(LED_LENGTH).setAnimation(LedBuilder.Animations.BLINKING_OFF_ON).setPrimaryColor(Color.kTan);
  }

  public enum LedState {
    DISABLED_NOT_READY(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kRed).setAnimation(Animations.BLINKING_OFF_ON).setFreq(4)),
    DISABLED_READY(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kGreen)),
    AUTO(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kBlue).setAnimation(Animations.BLINKING_OFF_ON)),
    AUTO_AUTOAIMING(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kBlue).setSecondaryColor(Color.kGreen).setAnimation(Animations.BLINKING_BETWEEN_COLORS)),
    AUTO_SEEKING(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kBlue).setSecondaryColor(Color.kOrange).setAnimation(Animations.BLINKING_BETWEEN_COLORS)),

    TELEOP_IDLE(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kPink)),
    //TELEOP_SEEKING(),
    TELEOP_AUTOAIM_PREPING(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kGreen).setAnimation(Animations.BLINKING_OFF_ON)),
    TELEOP_AUTOAIM_READY(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kGreen)),
    TELEOP_AUTOAIMING_OUTOFRANGE(new LedBuilder(LED_LENGTH).setPrimaryColor(Color.kGreen).setSecondaryColor(Color.kGreen).setAnimation(Animations.BLINKING_BETWEEN_COLORS))
    ;


    public LedBuilder builder;

    private LedState(LedBuilder builder) {
      this.builder = builder;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(ledstate.createBuffer());

    //
  }
}
