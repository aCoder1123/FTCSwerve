package org.firstinspires.ftc.teamcode.util;

public class SwerveDrivetrain {
	private double wheelSeparation = 0;
	public double wheelDiameter = 0;
	public double gearing = 0;
	public double maxSpeed = 0;
	private final double angleThreshold = 15;
	private final SwerveModule[] modules;

	public SwerveDrivetrain(SwerveModule[] modules) {
		if (modules.length != 2) {
			throw new Error("Drivetrain initialized with incorrect number of modules. Please initialize with 2 modules of type \"SwerveModule\".");
		}
		this.modules = modules;
	}

	public SwerveDrivetrain withWheelSeparation(double wheelSeparation) {
		this.wheelSeparation = wheelSeparation;
		return this;
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
		return 0;
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
		
		double v_rot = robotVelocity[2] * this.maxSpeed * 2 * Math.PI * (this.wheelSeparation / 2);
		double theta;

		if (robotVelocity[1] != 0) {
			theta = Math.atan(robotVelocity[0] / robotVelocity[1]);
		} else {
			theta = 0;
		}

		double wheel_1_x = robotVelocity[0] * this.maxSpeed + v_rot * Math.cos(theta);
		double wheel_1_y = robotVelocity[1] * this.maxSpeed + v_rot * Math.sin(theta);
		double wheel_1_v = Math.sqrt(wheel_1_y * wheel_1_y + wheel_1_x * wheel_1_x);
		double wheel_1_theta = Math.toDegrees(Math.atan(wheel_1_y / wheel_1_x));
		if (wheel_1_x == 0) {
			wheel_1_theta = 0;
		}
		states[0] = new ModuleState(wheel_1_v / (Math.PI * this.wheelDiameter) 
				* this.gearing,
				wheel_1_theta + this.getHeadingDegrees());

		double wheel_2_x = robotVelocity[0] * this.maxSpeed - v_rot * Math.cos(theta);
		double wheel_2_y = robotVelocity[1] * this.maxSpeed - v_rot * Math.sin(theta);
		double wheel_2_v = Math.sqrt(wheel_2_y * wheel_2_y + wheel_2_x * wheel_2_x);
		double wheel_2_theta = Math.toDegrees(Math.atan(wheel_2_y / wheel_2_x));
		if (wheel_2_x == 0) {
			wheel_2_theta = 0;
		}
		states[1] = new ModuleState(wheel_2_v * this.gearing / (Math.PI * this.wheelDiameter),
				wheel_2_theta + this.getHeadingDegrees());

		if (this.modules[0].getAngleDifference(states[0].theta) > this.angleThreshold || this.modules[1].getAngleDifference(states[1].theta) > this.angleThreshold) {
			states[0].rpm = 0;
			states[1].rpm = 0;
		}
		
		return states;
	}

	public void setStates(ModuleState[] states) {
		this.modules[0].setState(states[0]);
		this.modules[1].setState(states[1]);
	}

	public void driveWithJoysticks(double x, double y, double theta) {
		this.setStates(this.calculateModuleStates(this.normalizeRobotVelocity(x, y, theta)));
	}

}