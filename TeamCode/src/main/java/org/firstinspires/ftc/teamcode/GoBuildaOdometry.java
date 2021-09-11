package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="GoBuilda Drive Odometry Java")
public class GoBuildaOdometry extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        float LeftStickY;
        float LeftStickX;
        float RightStickX;
        double LeftStickYProcessed;
        double LeftStickXProcessed;
        double RightStickXProcessed;

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

        float x = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        // Assign variables to corresponding hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor = hardwareMap.get(DcMotor.class, "BR");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialization Code
        double powerMultiplier = 1;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup the odometry system
        // TODO: change all numbers from placeholder values to the values on the robot
        // Set the locations of the wheels on the robot
        Translation2d frontLeftLocation = new Translation2d(1,1);
        Translation2d frontRightLocation = new Translation2d(1,-1);
        Translation2d backLeftLocation = new Translation2d(-1,1);
        Translation2d backRightLocation = new Translation2d(-1,-1);
        // Create a mecanum kinematics object from the wheel locations
        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation);
        // Create a chassis speeds object relative to the field - format is (desired speed relative to one wall, desired speed relative to second wall, desired rotation speed in radians, current angle)
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(1.0,1.0,1.0,Rotation2d.fromDegrees(45.0));
        // Create an object to hold the angle of the robot as reported by the IMU
        Rotation2d gyroAngle = Rotation2d.fromDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);

        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        // Create an odometry object - format is (kinematics object, gyro heading, start position on field)
        MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, gyroAngle, new Pose2d(5, 5, new Rotation2d()));

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
