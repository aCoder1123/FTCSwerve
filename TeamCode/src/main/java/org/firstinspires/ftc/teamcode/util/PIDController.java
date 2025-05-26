package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public final double kP;
    public final double kI;
    public final double kD;
    private double tolerance;

    private double setpoint;
    private double integral = 0;
    private double lastTime = -1;
    private double lastError;
    private Double continuousInputMin;
    private Double continuousInputMax;
    private final ElapsedTime runtime;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.runtime = new ElapsedTime();
    }

    public void setSetpoint(double setpoint) {
        this.runtime.reset();
        this.setpoint = setpoint;
    }
    public void setTolerance(double tolerance) {this.tolerance = tolerance;}
    public void enableContinuousInput(double min, double max) {
        this.continuousInputMin = min;
        this.continuousInputMax = max;
    }

    public double calculate(double input) {
        double error = this.getError(input);
        if (Math.abs(error) < this.tolerance) {
            this.lastTime = this.runtime.milliseconds();
            this.lastError = error;
            return 0;
        }

        double timeInterval = this.lastTime != -1 ? (this.runtime.milliseconds() - this.lastTime)/1000 : 0;
        this.lastTime = this.runtime.milliseconds();
        this.integral += error * timeInterval;

       double output = this.kP * error;
        if (timeInterval != 0) {
            output += this.kI * this.integral + this.kD * ((error - this.lastError) / timeInterval);
        }

        this.lastError = error;
        return output;
    }

    public double getError(double measurement) {
        if (this.continuousInputMin == null) {
            return this.setpoint - measurement;
        } else {
            double nonCont = this.setpoint - measurement;
            double cont;
            if (this.setpoint < measurement) {
                cont = (this.continuousInputMax - measurement) + (this.setpoint - this.continuousInputMin);
            } else cont = (this.continuousInputMin - measurement) + (this.setpoint - this.continuousInputMax);
            if (Math.abs(nonCont) < Math.abs(cont)) {
                return nonCont;
            } else {
                return cont;
            }
        }
    }

}
