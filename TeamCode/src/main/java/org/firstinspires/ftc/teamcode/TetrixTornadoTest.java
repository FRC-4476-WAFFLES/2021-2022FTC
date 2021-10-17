package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Tetrix Tornado Test")
public class TetrixTornadoTest extends LinearOpMode {
    private DcMotor tetrixMotor;
    double power;

    @Override
    public void runOpMode() {
        tetrixMotor = hardwareMap.get(DcMotor.class, "tetrixMotor");
        tetrixMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for game to start
        waitForStart();

        while (opModeIsActive()) {
            power = gamepad1.left_stick_y;

            if (gamepad1.x) {
                power = 0;
            } else if (gamepad1.y) {
                power = 0.33;
            } else if (gamepad1.b) {
                power = 0.66;
            } else if (gamepad1.a) {
                power = 1;
            }

            tetrixMotor.setPower(power);

            telemetry.addData("Motor power", power);
            telemetry.update();
        }
    }
}
