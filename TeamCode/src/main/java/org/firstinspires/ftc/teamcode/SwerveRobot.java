package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.util.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.SwerveModule;

@TeleOp(name = "Drive With Joysticks", group = "Linear OpMode")

public class SwerveRobot extends LinearOpMode {

	private final ElapsedTime runtime = new ElapsedTime();

    public SwerveModule frontSwerveModule;
	public SwerveModule backSwerveModule;

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

		drivetrain = new SwerveDrivetrain(new SwerveModule[]{frontSwerveModule, backSwerveModule})
				.withGearing(DrivetrainConstants.gearing)
				.withMaxSpeed(DrivetrainConstants.maxSpeed)
				.withWheelDiameter(DrivetrainConstants.wheelDiameter)
				.withWheelSeparation(DrivetrainConstants.wheelSeparation);

		waitForStart();
		runtime.reset();

		telemetry.addData("Status", "Running");
		telemetry.update();

		while (opModeIsActive()) {
			/*
			 frontSwerveModule.runMotors(-this.gamepad1.left_stick_y,
			 -this.gamepad1.left_stick_x);
			 backSwerveModule.runMotors(-this.gamepad1.right_stick_y,
			 -this.gamepad1.right_stick_x);
			*/
			drivetrain.driveWithJoysticks(this.gamepad1.left_stick_y, -this.gamepad1.left_stick_x, -this.gamepad1.right_stick_x);

			telemetry.addData("FrontModuleState", frontSwerveModule.getState());
			telemetry.addData("FrontModuleSetpoint", frontSwerveModule.setpoint);
			telemetry.addData("BackModule", backSwerveModule.getEncoder().getVoltage());
			telemetry.addData("BackModuleSetpoint", backSwerveModule.setpoint);
			telemetry.update();

		}
	}

}
