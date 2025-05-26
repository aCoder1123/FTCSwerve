package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveModule {
	private final DcMotorEx motorOne;
	private final DcMotorEx motorTwo;
	private final AnalogInput encoder;
	private final double offset;
	public ModuleState setpoint;
	public final PIDController turnController;
	public final PIDController driveController;

	/**
	 * @param encoder encoder used on the swerve module
	 * @param encoderOffset offset in degrees to apply to make 0° point forward
	 * @param motorOne one of the motors on the module. Either one.
	 * @param motorTwo one of the motors on the module. Either one.
	 */
	public SwerveModule(AnalogInput encoder, double encoderOffset, DcMotorEx motorOne, DcMotorEx motorTwo) {
		this.motorOne = motorOne;
		this.motorTwo = motorTwo;
		this.encoder = encoder;
		this.offset = encoderOffset;

		turnController = new PIDController(kTurnP, kTurnI, kTurnD);
		turnController.setTolerance(1.5);
		turnController.enableContinuousInput(0, 360);

		this.motorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		this.motorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

		driveController = new PIDController(kDriveP, kDriveI, kDriveD);

	}

	/**
	 * Returns the angle the wheel is about the turning axis. Doesn't imply direction of travel.
	 * @return the angle in degrees
	 */
	public double getTurningAngle() {
		return (encoder.getVoltage() * (360. / encoder.getMaxVoltage()) - offset + 360) % 360;
	}

	public double getWheelRPM() {
		return gearing * (this.motorOne.getVelocity(AngleUnit.DEGREES) - this.motorTwo.getVelocity(AngleUnit.DEGREES))/360 * 60; //rpm = to the difference between motor speeds, converted to rpm times the gearing
	}

	public ModuleState getState() {
		return new ModuleState( this.getWheelRPM(), this.getTurningAngle());
	}

	public void runMotors(double motorOnePower, double motorTwoPower) {
		motorOne.setPower(motorOnePower);
		motorTwo.setPower(motorTwoPower);
	}

	public void setState(ModuleState state) {
		this.turnController.setSetpoint(state.theta);
		if (Math.abs(Math.abs(this.turnController.getError(this.getTurningAngle()))) > 90) {
			state.theta = (state.theta + 180) % 360;
			state.rpm *= -1;
			this.turnController.setSetpoint(state.theta);
		}
		this.driveController.setSetpoint(state.rpm/gearing);
		double drivePower = this.driveController.calculate(this.getWheelRPM());
		double turnPower = this.turnController.calculate(this.getTurningAngle());
		if (Math.abs(drivePower) > (1-Math.abs(turnPower))) {
			drivePower = Math.copySign(1-Math.abs(turnPower), drivePower);
		}
		this.setpoint = state;
		this.runMotors(drivePower + turnPower, -drivePower + turnPower);
	}
}
