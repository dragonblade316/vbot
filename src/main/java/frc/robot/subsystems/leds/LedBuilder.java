package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedBuilder {

    public enum Animations {
        STATIC,
        BLINKING_BETWEEN_COLORS,
        BLINKING_OFF_ON,
        ALTERNATING_TRAIN,
    }

    private Animations animation = Animations.STATIC;
    private Color primaryColor = Color.kWhite;
    private Color secondaryColor = Color.kBlack;
    private int freq = 1;
    private int counter = 0;
    private int buffer_length;

    public LedBuilder(int buffer_length) {
        this.buffer_length = buffer_length;
    }

    public LedBuilder setAnimation(Animations animation) {
        this.animation = animation;
        return this;
    }

    public LedBuilder setPrimaryColor(Color color) {
        primaryColor = color;
        return this;
    }

    public LedBuilder setSecondaryColor(Color color) {
        secondaryColor = color;
        return this;
    }

    public LedBuilder setFreq(int freq) {
        if (freq <= 0) {
            throw new IllegalArgumentException("freq can not be <= zero");
        }
        this.freq = freq;
        return this;
    }

    private AddressableLEDBuffer animationStatic() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(buffer_length);
        for (int i = 0; i == buffer.getLength()-1; i++) {
            buffer.setLED(i, primaryColor);
        }
        return buffer;
    }

    private AddressableLEDBuffer animationBlinkingOffOn() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(buffer_length);
        Color col;
        if (Math.signum(Math.sin(((counter-0.1)*Math.PI)/freq)) == 1) {
            col = primaryColor;
        } else {
            col = Color.kBlack;
        }


        for (int i = 0; i == buffer.getLength()-1; i++) {
            buffer.setLED(i, primaryColor);
        }
        return buffer;
    }

    private AddressableLEDBuffer animationBlinkingBetweenColors() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(buffer_length);
        Color col;
        if (Math.signum(Math.sin(((counter-0.1)*Math.PI)/freq)) == 1) {
            col = primaryColor;
        } else {
            col = secondaryColor;
        }


        for (int i = 0; i == buffer.getLength()-1; i++) {
            buffer.setLED(i, primaryColor);
        }
        return buffer;
    }

    private AddressableLEDBuffer animationAlternatingTrain() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(buffer_length);
        Color col;
        if (Math.signum(Math.sin(((counter-0.1)*Math.PI)/freq)) == 1) {
            col = primaryColor;
        } else {
            col = Color.kBlack;
        }


        for (int i = 0; i == buffer.getLength()-1; i++) {
            var num = Math.signum(Math.sin(((i+counter-0.1)*Math.PI)/freq));

            if (num < 0) {
                buffer.setLED(i, primaryColor);
            } else {
                buffer.setLED(i, secondaryColor);
            }
        }
        return buffer;
    }

    public AddressableLEDBuffer createBuffer() {
        AddressableLEDBuffer buffer;
        counter++;

        switch (animation) {
            case STATIC:
                buffer = animationStatic();
                break;
            case BLINKING_OFF_ON: 
                buffer = animationBlinkingOffOn();
                break;
            case BLINKING_BETWEEN_COLORS:
                buffer = animationBlinkingBetweenColors();
                break;
            case ALTERNATING_TRAIN:
                buffer = animationAlternatingTrain();
                break;
            default:
                buffer = new AddressableLEDBuffer(buffer_length);
                break;
        }

        return buffer;
    }  
}
