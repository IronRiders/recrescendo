package org.ironriders.lights;

import org.ironriders.lib.Constants;

import edu.wpi.first.wpilibj2.command.Command;

public class LightsCommands {

    private LightsSubsystem lights;

    public LightsCommands(LightsSubsystem lights) {
        this.lights = lights;

        lights.publish("Set to OFF", setLights(Constants.Lights.State.OFF));
        lights.publish("Set to GAY", setLights(Constants.Lights.State.GAY));
        lights.publish("Set to RAINBOW", setLights(Constants.Lights.State.RGB));
        lights.publish("Set to TRANS", setLights(Constants.Lights.State.TRANS));
        lights.publish("Set to ENBY", setLights(Constants.Lights.State.NONBINARY));

    }

    public Command setLights(Constants.Lights.State state) {
        return lights.defer(() -> {
            return new Command() {
                public void execute() {
                    lights.setLightState(state);
                }
            };
        });
    }
}
