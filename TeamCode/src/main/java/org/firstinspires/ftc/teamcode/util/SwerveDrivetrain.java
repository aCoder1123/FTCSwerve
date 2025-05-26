package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.*;

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

	/**
	 * Gets the heading as read from the IMU inside the control hub. 0° is forward, angle increases counter-clockwise
	 * @return the angle in degrees
	 */
	public double getHeadingDegrees() {
		return (360 + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + IMUOffset) % 360;
	}

	/**
	 * Normalizes the magnitude of the velocity vector to 1
	 * @param x x velocity -1 to 1. Positive values indicate right.
	 * @param y y velocity -1 to 1. Positive values indicate forwards.
	 * @param omega rotational velocity -1 to 1. Positive values indicate counter-clockwise.
	 * @return Normalized velocities as a list
	 */
	public double[] normalizeRobotVelocity(double x, double y, double omega) {
		double[] normalizedVelocities = new double[] { x, y, omega };
		double desiredWheelSpeed = Math.sqrt(x * x + y * y) + omega;
		if (desiredWheelSpeed > 1) {
			for (int i = 0; i < 3; i++) {
				normalizedVelocities[i] = normalizedVelocities[i] * (1 / desiredWheelSpeed);
			}
		}

		return normalizedVelocities;
	}

	public ModuleState[] calculateModuleStates(double[] robotVelocity) {
		ModuleState[] states = new ModuleState[2];

		if (robotVelocity[0] == 0 && robotVelocity[1] == 0 && robotVelocity[2] == 0) {
			states[0] = new ModuleState(0, modules[0].getTurningAngle());
			states[1] = new ModuleState(0, modules[1].getTurningAngle());
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

	public ModuleState calculateModuleStateTranslation(double[] robotVelocity) {
		double wheelRPM = motorMaxRpm * gearing * Math.sqrt(robotVelocity[0] * robotVelocity[0] + robotVelocity[1] * robotVelocity[1]);
		double angle;
		if (robotVelocity[0] == 0 && robotVelocity[1] == 0) {
			return new ModuleState(0, modules[0].getTurningAngle());
		} else if (robotVelocity[0] == 0) {
			angle = (robotVelocity[1] > 0 ? 0 : 180);
		} else if (robotVelocity[1] == 0) {
			angle = (robotVelocity[0] > 0 ? 90 : 270);
		} else {
			angle = ((Math.toDegrees(Math.atan(robotVelocity[1]/robotVelocity[0])) + 360 ) % 360);
			if (robotVelocity[0] < 0) {
				angle = (angle + 180) % 360; //flipping angle to correct quadrant if necessary
			}

			angle = (angle + 90) % 360; // 0° is forward
		}
		angle = (angle - this.getHeadingDegrees() + 360) % 360; // apply rotational offset for field relative and ensure angle still in range

		return new ModuleState(wheelRPM, angle); //both wheels are the same, only returns one state
	}

	public void setStates(ModuleState[] states) {
		this.modules[0].setState(states[0]);
		this.modules[1].setState(states[1]);
	}

	public void setStates(ModuleState state) {
		this.modules[0].setState(state);
		this.modules[1].setState(state);
	}

	public void driveWithJoysticks(double x, double y, double theta) {
		ModuleState[] states = this.calculateModuleStates(this.normalizeRobotVelocity(x, y, theta));
		if (states != null) {
			this.setStates(states);
		}
	}
	public void translateWithJoysticks(double x, double y) {
		ModuleState state = this.calculateModuleStateTranslation(this.normalizeRobotVelocity(x, y, 0));
		if (state != null) {
			this.setStates(state);
		}
	}

	public void pointAtDegrees(double theta) {
		theta = theta % 360;
		ModuleState state = new ModuleState(0, theta);
		this.setStates(state);
	}

}