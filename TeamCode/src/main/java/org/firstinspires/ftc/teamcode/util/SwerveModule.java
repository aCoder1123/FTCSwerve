package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;

public class SwerveModule {
	private final DcMotor motorOne;
	private final DcMotor motorTwo;
	private final AnalogInput encoder;
	private final double offset;
	public ModuleState setpoint;
	private final PIDController turnController;

	public SwerveModule(AnalogInput encoder, double encoderOffset, DcMotor motorOne, DcMotor motorTwo) {
		this.motorOne = motorOne;
		this.motorTwo = motorTwo;
		this.encoder = encoder;
		this.offset = encoderOffset;

		turnController = new PIDController(.01, 0, 0);
	}

	public AnalogInput getEncoder() {
		return encoder;
	}

	public double getAngle() {
		return encoder.getVoltage() * (360. / encoder.getMaxVoltage()) - offset;
	}

	public double getAngleDifference(double angle) {
		return Math.min(Math.abs(this.getAngle() - angle),
				Math.min(Math.abs((this.getAngle() + 360) - angle),
						Math.abs(this.getAngle() - (angle + 360))));
	}

	public ModuleState getState() {
		return new ModuleState(DrivetrainConstants.motorMaxRpm * DrivetrainConstants.gearing * (this.motorOne.getPower() - this.motorTwo.getPower()),
				this.getAngle());
	}

	public void runMotors(double motorOnePower, double motorTwoPower) {
		motorOne.setPower(motorOnePower);
		motorTwo.setPower(motorTwoPower);
	}

	public void setState(ModuleState state) {
		double angleDiff = this.getAngleDifference(state.theta);
		
		if (angleDiff > 90) {
			this.setpoint = new ModuleState(state.rpm * -1, (state.theta + 180) % 360);
		} else {
			this.setpoint = state;
		}
		
		double power = state.rpm * (DrivetrainConstants.gearing / DrivetrainConstants.motorMaxRpm);
		this.turnController.setSetpoint(this.setpoint.theta);
		double turnPower = this.turnController.calculate(this.getAngle());
		if (Math.abs(turnPower) > (1-Math.abs(power))) {
			turnPower = Math.copySign(1-Math.abs(power), turnPower);
		}
		
		this.runMotors(power + turnPower, -power + turnPower);
	}
}
