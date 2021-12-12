package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

@TeleOp(name = "GoBilda TeleOp")
public class GoBildaTeleOp extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private MotorEx elevatorMotor;
    private MotorEx angleMotor;
    private MotorEx intakeMotor;

    private ServoEx lockServo;

    private GyroEx gyro;

    private DigitalChannel elevatorLimit;

    private GamepadEx driverJoystick;
    private GamepadEx operatorJoystick;

    private MecanumDrive drive;

    //private ElevatorSubsystem elevator;

    @Override
    public void runOpMode(){
        double powerMultiplier = 1;

        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        elevatorMotor = new MotorEx(hardwareMap, "intakeRaise");
        intakeMotor = new MotorEx(hardwareMap, "intakeSpin");
        angleMotor = new MotorEx(hardwareMap, "Elevator");

        lockServo = new SimpleServo(hardwareMap, "Lock", 0, 90, AngleUnit.DEGREES);

        gyro = new RevIMU(hardwareMap, "imu");

        elevatorLimit = hardwareMap.get(DigitalChannel.class, "intakeRaiseLimit");

        driverJoystick = new GamepadEx(gamepad1);
        operatorJoystick = new GamepadEx(gamepad2);

        //elevator = new ElevatorSubsystem(elevatorMotor, angleMotor, lockServo, elevatorLimit);

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorMotor.setRunMode(Motor.RunMode.RawPower);

        gyro.init();

        drive = new MecanumDrive(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor);

        //elevator.initialize();

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

            /*
            if (operatorJoystick.getButton(GamepadKeys.Button.A)){
                elevator.goToTeleOp(ElevatorSubsystem.Levels.INTAKE);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.B)){
                elevator.goToTeleOp(ElevatorSubsystem.Levels.L1);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.X)){
                elevator.goToTeleOp(ElevatorSubsystem.Levels.L2);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.Y)){
                elevator.goToTeleOp(ElevatorSubsystem.Levels.CAROUSEL);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                elevator.goToTeleOp(ElevatorSubsystem.Levels.L3);
            }

            elevator.update();*/

            double intakePower = operatorJoystick.getRightY();
            double elevatorPower = operatorJoystick.getLeftY();

            if (!elevatorLimit.getState()){
                elevatorPower = Math.max(0, elevatorPower);
            }

            drive.driveFieldCentric(
                    driverJoystick.getLeftX() * powerMultiplier,
                    driverJoystick.getLeftY() * powerMultiplier,
                    driverJoystick.getRightX() * powerMultiplier,
                    gyro.getHeading() + 180);

            intakeMotor.set(-intakePower);
            elevatorMotor.set(elevatorPower);

            telemetry.addData("Limit Switch State", elevatorLimit.getState());

            telemetry.addData("Elevator Motor Current Position", elevatorMotor.getCurrentPosition());
            telemetry.addData("Driver LeftJoyX", driverJoystick.getLeftX());
            telemetry.addData("Driver LeftJoyY", driverJoystick.getLeftY());
            telemetry.addData("Driver RightJoyX", driverJoystick.getRightX());
            telemetry.addData("Absolute Heading", gyro.getAbsoluteHeading());
            telemetry.addData("Heading", gyro.getHeading());

            telemetry.addLine("");
            telemetry.addData("Operator RightJoyY", intakePower);

            telemetry.update();
        }
    }
}
