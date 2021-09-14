package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class MecanumNavigation extends SubsystemBase {
    private ElapsedTime runtime = new ElapsedTime();

    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;

    private final BNO055IMU imu;

    public MecanumNavigation(MotorEx frontLeftMotor, MotorEx frontRightMotor, MotorEx backLeftMotor, MotorEx backRightMotor, BNO055IMU imu, double xStart, double yStart, double omegaStart){
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.imu = imu;
    }

    public void initialize(){
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

        // Set motors to run at a specified velocity
        frontLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        frontRightMotor.setRunMode(Motor.RunMode.VelocityControl);
        backLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        backRightMotor.setRunMode(Motor.RunMode.VelocityControl);

        // Invert the direction of the left motors
        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);

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
        frontLeftMotor.setDistancePerPulse(0.59);
        frontRightMotor.setDistancePerPulse(0.59);
        backLeftMotor.setDistancePerPulse(0.59);
        backRightMotor.setDistancePerPulse(0.59);
    }

    public void initializeDrivetrain(double xStart, double yStart, double omegaStart){
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
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(1.0,1.0,1.0, Rotation2d.fromDegrees(45.0));
        // Create an object to hold the angle of the robot as reported by the IMU
        Rotation2d gyroAngle = Rotation2d.fromDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);

        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, gyroAngle, new Pose2d(xStart, yStart, new Rotation2d(omegaStart)));
    }

    @Override
    public void periodic(){
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(frontLeftMotor.getVelocity(), frontRightMotor.getVelocity(), backLeftMotor.getVelocity(), backRightMotor.getVelocity());
        Rotation2d gyroAngle = Rotation2d.fromDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);

    }
}
