package org.firstinspires.ftc.teamcode.util;

public class ModuleState {
    public double rpm;
    public double theta;

    public ModuleState(double rpm, double theta) {
        this.rpm = rpm;
        this.theta = theta;
    }

    public ModuleState() {
    }

    public ModuleState fromArray(double[] state) {
        this.rpm = state[0];

        this.theta = state[1] < 0 ? state[1] + 360 : state[1];

        return this;
    }

    public ModuleState copyState(ModuleState state) {
        this.rpm = state.rpm;
        this.theta = state.theta;
        return this;
    }

    public double speed() {
        return this.rpm;
    }

    public double rotations() {
        return this.theta / 360;
    }

    public double radians() {
        return this.theta * Math.PI / 180;
    }

    public double degrees() {
        return this.theta;
    }

    public double[] asArray() {
        return new double[] { this.rpm, this.theta };
    }

    public String toString() {
        return "Module State(" + this.rpm + ", " + this.theta + ")";
    }

    // public boolean valid() {
    // return this.rpm != null && this.theta != undefined;
    // }

    public static ModuleState optimizeState(ModuleState desiredState, ModuleState currentsState) {
        ModuleState optimizedState = new ModuleState().fromArray(new double[] { desiredState.rpm, 0 });

        if (Math.abs(desiredState.theta - currentsState.theta) > 180) {
            if (Math.abs((desiredState.theta + 360) - currentsState.theta) > 90) {
                optimizedState.rpm *= -1;
                optimizedState.theta = (desiredState.theta + 180) % 360;
            }

        } else {
            optimizedState.theta = desiredState.theta;
        }

        return optimizedState;
    }
}
