package org.firstinspires.ftc.teamcode.util;

public class PIDController {
    public final double kP;
    public final double kI;
    public final double kD;

    private double setpoint;
    private double integral = 0;
    private double lastTime = -1;
    private double lastInput;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return this.setpoint;
    }

    public double calculate(double input, double currentTime) {
        double difference = this.setpoint - input;
        if (this.lastTime != -1) {
            double timeInterval = currentTime - this.lastTime;
            this.lastTime = currentTime;
            this.integral += difference * timeInterval;
            double output = this.kP * difference + this.kI * this.integral + this.kD * ((input - this.lastInput) / timeInterval);
            this.lastInput = input;
    
            return output;
        } else {
            this.lastTime = currentTime;
            this.lastInput = input;

            return this.kP * difference;
        }
    }

    public double calculate(double input) {
        double difference = this.setpoint - input;
        double output = this.kP * difference;
        this.lastInput = input;

        return output;
    }

}
