package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GoBuilda TeleOp")
public class GoBuildaTeleOp extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private MotorEx elevatorMotor;
    private MotorEx intakeMotor;

    private GyroEx gyro;

    private GamepadEx driverJoystick;
    private GamepadEx operatorJoystick;

    private MecanumDrive drive;

    @Override
    public void runOpMode(){
        double powerMultiplier = 1;

        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        elevatorMotor = new MotorEx(hardwareMap, "intakeRaise");
        intakeMotor = new MotorEx(hardwareMap, "intakeSpin");

        gyro = new RevIMU(hardwareMap, "imu");

        driverJoystick = new GamepadEx(gamepad1);
        operatorJoystick = new GamepadEx(gamepad2);

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        elevatorMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        gyro.init();

        drive = new MecanumDrive(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor);

        waitForStart();

        while (opModeIsActive()) {
            if (driverJoystick.getButton(GamepadKeys.Button.X)) {
                powerMultiplier = 0.33;
            } else if (driverJoystick.getButton(GamepadKeys.Button.Y)) {
                powerMultiplier = 0.66;
            } else if (driverJoystick.getButton(GamepadKeys.Button.B)) {
                powerMultiplier = 1;
            }

            if (driverJoystick.getButton(GamepadKeys.Button.START) && driverJoystick.getButton(GamepadKeys.Button.BACK)){
                gyro.reset();
            }

            double elevatorPower = operatorJoystick.getLeftY();
            double intakePower = operatorJoystick.getRightY();

            drive.driveFieldCentric(
                    driverJoystick.getLeftX() * powerMultiplier,
                    driverJoystick.getLeftY() * powerMultiplier,
                    driverJoystick.getRightX() * powerMultiplier,
                    gyro.getHeading() - 90);

            elevatorPower = Math.min(elevatorPower, 0.4);

            elevatorMotor.set(elevatorPower);

            intakeMotor.set(-intakePower);

            telemetry.addData("Elevator Motor Current Position", elevatorMotor.getCurrentPosition());
            telemetry.addData("Driver LeftJoyX", driverJoystick.getLeftX());
            telemetry.addData("Driver LeftJoyY", driverJoystick.getLeftY());
            telemetry.addData("Driver RightJoyX", driverJoystick.getRightX());
            telemetry.addData("Absolute Heading", gyro.getAbsoluteHeading());
            telemetry.addData("Heading", gyro.getHeading());

            telemetry.addLine("");
            telemetry.addData("Operator LeftJoyY", elevatorPower);
            telemetry.addData("Operator RightJoyY", intakePower);

            telemetry.update();
        }
    }
}
