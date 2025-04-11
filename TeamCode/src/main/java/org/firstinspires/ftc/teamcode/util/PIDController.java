package org.firstinspires.ftc.teamcode.util;

public class PIDController {
    public final double kP;
    public final double kI;
    public final double kD;
    private double tolerance;

    private double setpoint;
    private double integral = 0;
    private double lastTime = -1;
    private double lastInput;
    private Double continuousInputMin;
    private Double continuousInputMax;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    public void setTolerance(double tolerance) {this.tolerance = tolerance;}
    public void enableContinuousInput(double min, double max) {
        this.continuousInputMin = min;
        this.continuousInputMax = max;
    }

    public double calculate(double input, double currentTime) {
        double error = this.getError(input);
        if (Math.abs(error) < this.tolerance) return 0;

        if (this.lastTime != -1) {
            double timeInterval = currentTime - this.lastTime;
            this.lastTime = currentTime;
            this.integral += error * timeInterval;

            double nonCont = Math.abs(this.setpoint - input);
            double cont;
            if (this.setpoint < input) {
                cont = Math.abs(this.continuousInputMin - input) + Math.abs(this.setpoint - this.continuousInputMax);
            } else cont = Math.abs(this.continuousInputMax - input) + Math.abs(this.setpoint - this.continuousInputMin);

            double output;
            if (cont > nonCont) {
                output = this.kP * (this.setpoint - input) + this.kI * this.integral + this.kD * ((input - this.lastInput) / timeInterval);
            } else {
                output = this.kP * -Math.copySign(error, this.setpoint - input) + this.kI * this.integral + this.kD * ((input - this.lastInput) / timeInterval);
            }

            this.lastInput = input;

            return output;
        } else {
            this.lastTime = currentTime;

            return this.calculate(input);
        }
    }

    public double calculate(double input) {
        this.lastInput = input;
        double error = this.getError(input);
        if (Math.abs(error) < this.tolerance) return 0;

        double nonCont = Math.abs(this.setpoint - input);
        double cont;
        if (this.setpoint < input) {
            cont = Math.abs(this.continuousInputMin - input) + Math.abs(this.setpoint - this.continuousInputMax);
        } else cont = Math.abs(this.continuousInputMax - input) + Math.abs(this.setpoint - this.continuousInputMin);

        double output;
        if (cont > nonCont) {
            output = this.kP * (this.setpoint - input);
        } else {
            output = this.kP * -Math.copySign(error, this.setpoint - input);
        }

        return output;

    }

    public double getError(double measurement) {
        if (this.continuousInputMin == null) {
            return Math.abs(this.setpoint - measurement);
        } else {
            double nonCont = Math.abs(this.setpoint - measurement);
            double cont;
            if (this.setpoint < measurement) {
                cont = Math.abs(this.continuousInputMin - measurement) + Math.abs(this.setpoint - this.continuousInputMax);
            } else cont = Math.abs(this.continuousInputMax - measurement) + Math.abs(this.setpoint - this.continuousInputMin);

            return Math.min(nonCont, cont);
        }
    }

}
