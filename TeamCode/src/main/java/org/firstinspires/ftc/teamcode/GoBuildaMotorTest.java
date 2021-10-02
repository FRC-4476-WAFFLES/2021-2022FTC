package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test")
public class GoBuildaMotorTest extends LinearOpMode {
    private GamepadEx driverOp;
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;

    final double CPR = 2150.8; // Encoder ticks per wheel rotation
    final double WHEEL_DIAMETER = 101.6; // Wheel diameter in millimeters
    final double MM_PER_TICK = WHEEL_DIAMETER * 3.141592 / CPR; // Wheel distance traveled per encoder tick in millimeters
    final double METERS_TO_TICKS = (1/MM_PER_TICK)*1000; // Convert meters per second to ticks per second
    @Override
    public void runOpMode(){

        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        frontLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        frontRightMotor.setRunMode(Motor.RunMode.RawPower);
        frontLeftMotor.setDistancePerPulse(MM_PER_TICK);
        driverOp = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive()){
            double velocity = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            frontLeftMotor.setVelocity(velocity*METERS_TO_TICKS);
            frontRightMotor.set(velocity);
            telemetry.addData("Speed", velocity);
            telemetry.addData("Left Speed", frontLeftMotor.getVelocity());
            telemetry.addData("Right Speed", frontRightMotor.getVelocity());
            telemetry.update();
        }
    }
}
