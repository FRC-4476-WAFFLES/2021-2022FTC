package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="GoBuilda Drive Java")
public class GoBuildaJavaDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private Gyroscope imu;

    @Override
    public void runOpMode() {
        float LeftStickY;
        float LeftStickX;
        float RightStickX;
        double LeftStickYProcessed;
        double LeftStickXProcessed;
        double RightStickXProcessed;

        // Assign variables to corresponding hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor = hardwareMap.get(DcMotor.class, "BR");

        imu = hardwareMap.get(Gyroscope.class, "imu");

        // Initialization Code
        double powerMultiplier = 1;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for game to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            LeftStickY = gamepad1.left_stick_y;
            LeftStickX = gamepad1.left_stick_x;
            RightStickX = gamepad1.right_stick_x;

            if (gamepad1.x) {
                powerMultiplier = 0.33;
            } else if (gamepad1.y) {
                powerMultiplier = 0.66;
            } else if (gamepad1.b) {
                powerMultiplier = 1;
            }

            LeftStickYProcessed = Math.signum(LeftStickY) * Math.pow(LeftStickY, 2);
            LeftStickXProcessed = Math.signum(LeftStickX) * Math.pow(LeftStickX, 2);
            RightStickXProcessed = Math.signum(RightStickX) * Math.pow(RightStickX, 2);

            frontLeftMotor.setPower(powerMultiplier * (-LeftStickYProcessed + LeftStickXProcessed + RightStickXProcessed));
            frontRightMotor.setPower(powerMultiplier * (-LeftStickYProcessed - LeftStickXProcessed - RightStickXProcessed));
            backLeftMotor.setPower(powerMultiplier * (-LeftStickYProcessed - LeftStickXProcessed + RightStickXProcessed));
            backRightMotor.setPower(powerMultiplier * (-LeftStickYProcessed + LeftStickXProcessed - RightStickXProcessed));

            telemetry.update();
        }
    }
}
