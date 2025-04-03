package org.ironriders.lights;

import org.ironriders.lib.Constants;

import edu.wpi.first.wpilibj2.command.Command;

public class LightsCommands {

    private LightsSubsystem lights;

    public LightsCommands(LightsSubsystem lights) {
        this.lights = lights;

        lights.publish("Set lights to off", setLights(Constants.Lights.State.OFF));
        lights.publish("Set lights to gay", setLights(Constants.Lights.State.GAY));
        lights.publish("Set lights to rainbow", setLights(Constants.Lights.State.RGB));
        lights.publish("Set lights to trans", setLights(Constants.Lights.State.TRANS));
        lights.publish("Set lights to non binary", setLights(Constants.Lights.State.NONBINARY));

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
