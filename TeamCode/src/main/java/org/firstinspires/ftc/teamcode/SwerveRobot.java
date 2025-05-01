package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");

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
			telemetry.addData("FrontModuleState", frontSwerveModule.getState().theta);
			telemetry.addData("FrontModuleSetpoint", frontSwerveModule.setpoint.theta);
			telemetry.addData("fError", frontSwerveModule.turnController.getError(frontSwerveModule.getAngle()));
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
