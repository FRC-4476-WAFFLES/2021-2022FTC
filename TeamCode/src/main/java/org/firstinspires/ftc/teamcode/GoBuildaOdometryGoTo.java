package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "GoBuilda Single Destination")
public class GoBuildaOdometryGoTo extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private BNO055IMU imu;

    double locationX = 0;
    double locationY = 0;
    double locationOmega = 0;

    final double locationXStart = 0;
    final double locationYStart = 0;
    final double locationOmegaStart = 0;

    final double locationXDestination = 0.5;
    final double locationYDestination = 0.5;
    final double locationOmegaDestination = 0;

    final double CPR = 2150.8; // Encoder ticks per wheel rotation
    final double WHEEL_DIAMETER = 101.6; // Wheel diameter in millimeters
    final double MM_PER_TICK = WHEEL_DIAMETER * 3.141592 / CPR; // Wheel distance traveled per encoder tick in millimeters
    final double METERS_TO_TICKS = (1 / MM_PER_TICK) * 1000; // Convert meters per second to ticks per second

    @Override
    public void runOpMode() {
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

        // Assign motor variables to corresponding hardware
        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        // Set motors to run at a specified velocity
        frontLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        frontRightMotor.setRunMode(Motor.RunMode.VelocityControl);
        backLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        backRightMotor.setRunMode(Motor.RunMode.VelocityControl);

        // Invert the direction of the left motors
        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        // Set the motors to brake on stop
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Reset the motor encoders
        frontLeftMotor.resetEncoder();
        frontRightMotor.resetEncoder();
        backLeftMotor.resetEncoder();
        backRightMotor.resetEncoder();

        // Set how far the wheels move in millimeters per encoder pulse
        frontLeftMotor.setDistancePerPulse(MM_PER_TICK);
        frontRightMotor.setDistancePerPulse(MM_PER_TICK);
        backLeftMotor.setDistancePerPulse(MM_PER_TICK);
        backRightMotor.setDistancePerPulse(MM_PER_TICK);

        // Setup the odometry system
        // TODO: change all numbers from placeholder values to the values on the robot
        // Set the locations of the wheels on the robot
        Translation2d frontLeftLocation = new Translation2d(1,1);
        Translation2d frontRightLocation = new Translation2d(1,-1);
        Translation2d backLeftLocation = new Translation2d(-1,1);
        Translation2d backRightLocation = new Translation2d(-1,-1);
        // Create a mecanum kinematics object from the wheel locations
        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation);
        // Create an object to hold the angle of the robot as reported by the IMU
        Rotation2d gyroAngle = Rotation2d.fromDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        // Create a chassis speeds object relative to the field - format is (desired speed relative to one wall, desired speed relative to second wall, desired rotation speed in radians, current angle)
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, gyroAngle);

        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        // Create an odometry object - format is (kinematics object, gyro heading, start position on field)
        MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, gyroAngle, new Pose2d(locationXStart, locationYStart, new Rotation2d(locationOmegaStart)));

        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for game to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double xVelocity;
            double yVelocity;
            double omegaVelocity;

            double frontLeftSpeed;
            double frontRightSpeed;
            double backLeftSpeed;
            double backRightSpeed;

            locationOmega = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            gyroAngle = Rotation2d.fromDegrees(locationOmega);

            odometry.updateWithTime(runtime.time(), gyroAngle, wheelSpeeds);

            locationX = odometry.getPoseMeters().getX();
            locationY = odometry.getPoseMeters().getY();

            if (locationX + 0.05 < locationXDestination) {
                xVelocity = 0.2;
            } else if (locationX - 0.05 > locationXDestination) {
                xVelocity = -0.2;
            } else {
                xVelocity = 0;
            }

            if (locationY + 0.05 < locationYDestination) {
                yVelocity = 0.2;
            } else if (locationY - 0.05 > locationYDestination) {
                yVelocity = -0.2;
            } else {
                yVelocity = 0;
            }

            if (locationOmega + 0.05 < locationOmegaDestination) {
                omegaVelocity = 0.2;
            } else if (locationOmega - 0.05 > locationOmegaDestination) {
                omegaVelocity = -0.2;
            } else {
                omegaVelocity = 0;
            }

            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omegaVelocity, gyroAngle);
            wheelSpeeds = kinematics.toWheelSpeeds(speeds);

            frontLeftSpeed = wheelSpeeds.frontLeftMetersPerSecond;
            frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
            backLeftSpeed = wheelSpeeds.rearLeftMetersPerSecond;
            backRightSpeed = wheelSpeeds.rearRightMetersPerSecond;

            frontLeftMotor.setVelocity(frontLeftSpeed * METERS_TO_TICKS);
            frontRightMotor.setVelocity(frontRightSpeed * METERS_TO_TICKS);
            backLeftMotor.setVelocity(backLeftSpeed * METERS_TO_TICKS);
            backRightMotor.setVelocity(backRightSpeed * METERS_TO_TICKS);

            telemetry.addData("FL Power", frontLeftSpeed);
            telemetry.addData("FR Power", frontRightSpeed);
            telemetry.addData("BL Power", backLeftSpeed * METERS_TO_TICKS);
            telemetry.addData("BR Power", backRightSpeed * METERS_TO_TICKS);

            telemetry.addData("Heading", locationOmega);
            telemetry.addData("X", locationX);
            telemetry.addData("Y", locationY);

            telemetry.update();
        }
    }
}
