package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "GoBuilda TeleOp")
public class GoBuildaTeleOp extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private MotorEx elevatorMotor;
    private MotorEx intakeMotor;

    private MecanumDrive drive;

    private GamepadEx driverJoystick;
    private GamepadEx operatorJoystick;

    private BNO055IMU imu;

    @Override
    public void runOpMode(){
        double powerMultiplier = 1;

        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        elevatorMotor = new MotorEx(hardwareMap, "Elevator");
        intakeMotor = new MotorEx(hardwareMap, "Intake");

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        drive = new MecanumDrive(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor);
        driverJoystick = new GamepadEx(gamepad1);
        operatorJoystick = new GamepadEx(gamepad2);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters imuParameters;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);

        waitForStart();

        while (opModeIsActive()){
            if (driverJoystick.getButton(GamepadKeys.Button.X)) {
                powerMultiplier = 0.33;
            } else if (driverJoystick.getButton(GamepadKeys.Button.Y)) {
                powerMultiplier = 0.66;
            } else if (driverJoystick.getButton(GamepadKeys.Button.B)) {
                powerMultiplier = 1;
            }

            drive.driveFieldCentric(driverJoystick.getLeftX() * powerMultiplier, driverJoystick.getLeftY() * powerMultiplier, driverJoystick.getRightX() * powerMultiplier, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);

            elevatorMotor.set(operatorJoystick.getLeftY());
            intakeMotor.set(operatorJoystick.getRightY());

            /*
            if (operatorJoystick.getButton(GamepadKeys.Button.A)){
                intakeMotor.set(1);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.B)){
                intakeMotor.set(-1);
            } else {
                intakeMotor.set(0);
            }*/

            telemetry.addData("LeftJoyX", driverJoystick.getLeftX());
            telemetry.addData("LeftJoyY", driverJoystick.getLeftY());
            telemetry.addData("RightJoyX", driverJoystick.getRightX());
            telemetry.addData("Yaw", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }
    }
}
