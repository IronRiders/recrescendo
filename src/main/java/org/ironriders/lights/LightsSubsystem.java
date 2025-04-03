package org.ironriders.lights;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironriders.lib.Constants;
import org.ironriders.lib.Constants.Identifiers;
import org.ironriders.lib.IronSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LightsSubsystem extends IronSubsystem {

    public LightsCommands commands = new LightsCommands(this);

    private final AddressableLED addressableLED = new AddressableLED(Identifiers.LIGHTING_STRIP_PORT);

    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.Lights.STRIP_LENGTH);

    private Constants.Lights.State lightState = Constants.Lights.State.OFF;

    private LEDPattern pattern = LEDPattern.solid(Color.kBlack);

    public LightsSubsystem() {
        addressableLED.setLength(buffer.getLength());
        addressableLED.start(); // maybe bad

        setColorToColorState(Constants.Lights.ColorState.GREEN);
    }

    @Override
    public void periodic() {
        switch (lightState) {
            case OFF:
                pattern = LEDPattern.solid(Color.kBlack);

            case RGB:
                pattern = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(
                        MetersPerSecond.of(Constants.Lights.State.RGB.scrollSpeed),
                        Constants.Lights.STRIP_DENSITY);

            case TRANS:
                pattern = LEDPattern
                        .gradient(LEDPattern.GradientType.kContinuous, new Color("#5BCEFA"), new Color("#F5A9B8"),
                                new Color("#FFFFFF"))
                        .scrollAtAbsoluteSpeed(MetersPerSecond.of(Constants.Lights.State.TRANS.scrollSpeed),
                                Constants.Lights.STRIP_DENSITY);

            case GAY:
                pattern = LEDPattern
                        .gradient(LEDPattern.GradientType.kContinuous, new Color("#FF6599"), new Color("#FF0000"),
                                new Color("#FF8E00"), new Color("#FFFF00"), new Color("#008E00"), new Color("#00C0C0"),
                                new Color("#400098"), new Color("#8E008E"))
                        .scrollAtAbsoluteSpeed(MetersPerSecond.of(Constants.Lights.State.GAY.scrollSpeed),
                                Constants.Lights.STRIP_DENSITY);
        }
        pattern.applyTo(buffer);
        addressableLED.setData(buffer);
    }

    public void setLightState(Constants.Lights.State state) {
        this.lightState = state;
    }

    public void setColorToColorState(Constants.Lights.ColorState state) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, state.r, state.g, state.b);
        }

        addressableLED.setData(buffer);
    }

    public void setRGB(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }

        addressableLED.setData(buffer);
    }

    public LightsCommands getCommands() {
        return commands;
    }
}
