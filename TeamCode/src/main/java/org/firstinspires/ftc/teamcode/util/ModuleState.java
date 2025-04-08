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
        this.theta = state[1];
        return this;
    }

    public double[] asArray() {
        return new double[] { this.rpm, this.theta };
    }

    public String toString() {
        return "Module State(" + this.rpm + ", " + this.theta + ")";
    }

}
