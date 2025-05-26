package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.util.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.SwerveModule;

import java.util.Arrays;

@TeleOp(name = "Drive With Joysticks", group = "Linear OpMode")

public class SwerveRobot extends LinearOpMode {

	private final ElapsedTime runtime = new ElapsedTime();

    public SwerveModule frontSwerveModule;
	public SwerveModule backSwerveModule;
	public IMU imu;

	public SwerveDrivetrain drivetrain;

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "fr");
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "br");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "bl");

        AnalogInput frontEncoder = hardwareMap.get(AnalogInput.class, "backEncoder");
        AnalogInput backEncoder = hardwareMap.get(AnalogInput.class, "frontEncoder");

		frontSwerveModule = new SwerveModule(frontEncoder, DrivetrainConstants.frontEncoderOffset, fr, fl);
		backSwerveModule = new SwerveModule(backEncoder, DrivetrainConstants.backEncoderOffset, br, bl);

		imu = hardwareMap.get(IMU.class, "imu");
		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.UP,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		imu.initialize(parameters);
		imu.resetYaw();

		drivetrain = new SwerveDrivetrain(new SwerveModule[]{frontSwerveModule, backSwerveModule}, imu)
				.withGearing(DrivetrainConstants.gearing)
				.withMaxSpeed(DrivetrainConstants.maxSpeed)
				.withWheelDiameter(DrivetrainConstants.wheelDiameter);

		waitForStart();
		runtime.reset();

		while (opModeIsActive()) {
//			drivetrain.driveWithJoysticks(this.gamepad1.left_stick_x, -this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);
			drivetrain.translateWithJoysticks(this.gamepad1.left_stick_x, -this.gamepad1.left_stick_y);

			if (gamepad1.options) {
				imu.resetYaw();
			}
			if (gamepad1.dpad_up) {
				drivetrain.pointAtDegrees(0);
			}
			if (gamepad1.dpad_down) {
				drivetrain.pointAtDegrees(180);
			}
			if (gamepad1.dpad_right) {
				drivetrain.pointAtDegrees(270);
			}
			if (gamepad1.dpad_left) {
				drivetrain.pointAtDegrees(90);
			}
			telemetry.addData("FrontModuleState", frontSwerveModule.getState().theta);
			telemetry.addData("FrontModuleSetpoint", frontSwerveModule.setpoint.theta);
			telemetry.addData("fError", frontSwerveModule.turnController.getError(frontSwerveModule.getTurningAngle()));
//			telemetry.addData("BackModuleState", backSwerveModule.getState());
//			telemetry.addData("BackModuleSetpoint", backSwerveModule.setpoint);
			telemetry.addData("vx", this.gamepad1.left_stick_x);
			telemetry.addData("vy", -this.gamepad1.left_stick_y);
			telemetry.addData("velocities", Arrays.toString(drivetrain.normalizeRobotVelocity(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x)));
			telemetry.addData("imu", drivetrain.getHeadingDegrees());
			telemetry.update();

		}
	}

}
