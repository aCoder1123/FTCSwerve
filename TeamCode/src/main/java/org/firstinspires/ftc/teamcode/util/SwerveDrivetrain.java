package org.firstinspires.ftc.teamcode.util;

//import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveDrivetrain {
	public double wheelDiameter = 0;
	public double gearing = 0;
	public double maxSpeed = 0;

	private final SwerveModule[] modules;
	private final IMU imu;

	public SwerveDrivetrain(SwerveModule[] modules, IMU imu) {
		if (modules.length != 2) {
			throw new Error("Drivetrain initialized with incorrect number of modules. Please initialize with 2 modules of type \"SwerveModule\".");
		}
		this.modules = modules;
		this.imu = imu;
	}

	public SwerveDrivetrain withWheelDiameter(double wheelDiameter) {
		this.wheelDiameter = wheelDiameter;
		return this;
	}

	public SwerveDrivetrain withGearing(double gearing) {
		this.gearing = gearing;
		return this;
	}

	public SwerveDrivetrain withMaxSpeed(double maxSpeed) {
		this.maxSpeed = maxSpeed;
		return this;
	}

	public double getHeadingDegrees() {
		return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	public double[] normalizeRobotVelocity(double x, double y, double omega) {
		double[] normalizedVelocities = new double[] { x, y, omega };
		double desiredWheelSpeed = Math.sqrt(x * x + y * y) + omega;
		if (desiredWheelSpeed > 1) {
			for (int i = 0; i < 3; i++) {
				normalizedVelocities[i] = normalizedVelocities[i] * (1 / desiredWheelSpeed);
			}
		}

		return normalizedVelocities; //[x - side to side, y - forward backward, theta - rotation (0° = straight forward)]
	}

	public ModuleState[] calculateModuleStates(double[] robotVelocity) {
		ModuleState[] states = new ModuleState[2];

		if (robotVelocity[0] == 0 && robotVelocity[1] == 0 && robotVelocity[2] == 0) {
			states[0] = new ModuleState(0, modules[0].getAngle());
			states[1] = new ModuleState(0, modules[1].getAngle());
			return states;
		}

		double v_rot = robotVelocity[2] * this.maxSpeed;
//		double theta;
//
//		if (robotVelocity[1] != 0) {
//			theta = Math.atan(robotVelocity[0] / robotVelocity[1]);
//			if (robotVelocity[0] < 0) {
//				theta += Math.toRadians(180);
//			}
//			if (robotVelocity[0] == 0) {
//				theta = Math.toRadians(180 + Math.copySign(90, robotVelocity[1]));
//			}
//		} else {
//			theta = 0;
//		}

		double wheel_1_x = robotVelocity[0] * this.maxSpeed + v_rot;// * Math.cos(theta + 45);
		double wheel_1_y = robotVelocity[1] * this.maxSpeed + v_rot;// * Math.sin(theta + 45);
		double wheel_1_v = Math.sqrt(wheel_1_y * wheel_1_y + wheel_1_x * wheel_1_x);
		double wheel_1_theta = Math.toDegrees(Math.atan(wheel_1_y / wheel_1_x));
		if (wheel_1_y < 0) wheel_1_theta += 180;

		if (wheel_1_x == 0) {
			if (wheel_1_y != 0) {
				wheel_1_theta = 180 - Math.copySign(90, robotVelocity[1]);
			} else wheel_1_theta = 0;
		}
		states[0] = new ModuleState(wheel_1_v / (Math.PI * this.wheelDiameter
				* this.gearing),
				wheel_1_theta + this.getHeadingDegrees());

		double wheel_2_x = robotVelocity[0] * this.maxSpeed - v_rot;// * Math.cos(theta + 45);
		double wheel_2_y = robotVelocity[1] * this.maxSpeed - v_rot;// * Math.sin(theta + 45);
		double wheel_2_v = Math.sqrt(wheel_2_y * wheel_2_y + wheel_2_x * wheel_2_x);
		double wheel_2_theta = Math.toDegrees(Math.atan(wheel_2_y / wheel_2_x));
		if (wheel_2_y < 0) wheel_2_theta += 180;

		if (wheel_2_x == 0) {
			if (wheel_2_y != 0) {
				wheel_2_theta = 180 - Math.copySign(90, robotVelocity[1]);
			} else wheel_2_theta = 0;
		}
		states[1] = new ModuleState(wheel_2_v / (Math.PI * this.wheelDiameter
				* this.gearing),
				wheel_2_theta + this.getHeadingDegrees());

//		if (this.modules[0].getAngleDifference(states[0].theta) > DrivetrainConstants.angleThreshold || this.modules[1].getAngleDifference(states[1].theta) > DrivetrainConstants.angleThreshold) {
//			states[0].rpm = 0;
//			states[1].rpm = 0;
//		}
		
		return states;
	}

	public void setStates(ModuleState[] states) {
		this.modules[0].setState(states[0]);
		this.modules[1].setState(states[1]);
	}

	public void driveWithJoysticks(double x, double y, double theta) {
		ModuleState[] states = this.calculateModuleStates(this.normalizeRobotVelocity(x, y, theta));
		if (states != null) {
			this.setStates(states);
		}
	}

}