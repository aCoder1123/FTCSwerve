package org.firstinspires.ftc.teamcode.util;

public class ModuleState {
    public double rpm; //RPM of the wheel
    public double theta; //Angle the wheel points to in degrees. Forward is 0°

    /**
     * @param rpm  RPM of the wheel
     * @param theta  Angle the wheel points to in degrees. Forward is 0°
     */
    public ModuleState(double rpm, double theta) {
        this.rpm = rpm;
        this.theta = theta;
    }

    public String toString() {
        return "Module State(" + this.rpm + ", " + this.theta + ")";
    }

}
