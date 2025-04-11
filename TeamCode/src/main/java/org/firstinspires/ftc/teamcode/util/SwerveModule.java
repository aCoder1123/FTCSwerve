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

		turnController = new PIDController(.035, 0, 0);
		turnController.setTolerance(1.5);
		turnController.enableContinuousInput(0, 360);
	}

	public double getAngle() {
		return encoder.getVoltage() * (360. / encoder.getMaxVoltage()) - offset;
	}

	public ModuleState getState() {
		return new ModuleState(DrivetrainConstants.motorMaxRpm * (this.motorOne.getPower() - this.motorTwo.getPower()),
				this.getAngle());
	}

	public void runMotors(double motorOnePower, double motorTwoPower) {
		motorOne.setPower(motorOnePower);
		motorTwo.setPower(motorTwoPower);
	}

	public void setState(ModuleState state) {
		this.turnController.setSetpoint(this.setpoint.theta);
		if (this.turnController.getError(this.getAngle()) > 90)  {
			this.setpoint.theta = (this.setpoint.theta + 180) % 360;
			this.setpoint.rpm *= -1;
			this.turnController.setSetpoint(this.setpoint.theta);
		}

		double power = state.rpm/DrivetrainConstants.motorMaxRpm;
		double turnPower = this.turnController.calculate(this.getAngle());
		if (Math.abs(power) > (1-Math.abs(turnPower))) {
			power = Math.copySign(1-Math.abs(turnPower), power);
		}
		
		this.runMotors(power + turnPower, -power + turnPower);
	}
}
