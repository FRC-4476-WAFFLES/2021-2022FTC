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

public class GoBuildaChassisSubsystem extends SubsystemBase {
    private ElapsedTime runtime = new ElapsedTime();

    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;

    private final BNO055IMU imu;

    final double CPR = 2150.8; // Encoder ticks per wheel rotation
    final double WHEEL_DIAMETER = 101.6; // Wheel diameter in millimeters
    final double MM_PER_TICK = WHEEL_DIAMETER * 3.141592 / CPR; // Wheel distance traveled per encoder tick in millimeters
    final double METERS_TO_TICKS = (1 / MM_PER_TICK) * 1000; // Convert meters per second to ticks per second

    MecanumDriveKinematics kinematics;
    Rotation2d gyroAngle;
    ChassisSpeeds speeds;
    MecanumDriveWheelSpeeds wheelSpeeds;
    MecanumDriveOdometry odometry;

    double vxMultiplier = 1;
    double vyMultiplier = 1;
    double vaMultiplier = 1;

    public GoBuildaChassisSubsystem(MotorEx frontLeftMotor, MotorEx frontRightMotor, MotorEx backLeftMotor, MotorEx backRightMotor, BNO055IMU imu){
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.imu = imu;
    }

    public void initialize(double xStart, double yStart, double omegaStart){
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
        kinematics = new MecanumDriveKinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation);
        // Create an object to hold the angle of the robot as reported by the IMU
        gyroAngle = Rotation2d.fromDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        // Create a chassis speeds object relative to the field - format is (desired speed relative to one wall, desired speed relative to second wall, desired rotation speed in radians, current angle)
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, gyroAngle);

        wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        // Create an odometry object - format is (kinematics object, gyro heading, start position on field)
        odometry = new MecanumDriveOdometry(kinematics, gyroAngle, new Pose2d(xStart, yStart, new Rotation2d(omegaStart)));
    }

    public void setVelocityMultipliers(double xVelocityMultiplier, double yVelocityMultiplier, double aVelocityMultiplier){
        vxMultiplier = xVelocityMultiplier;
        vyMultiplier = yVelocityMultiplier;
        vaMultiplier = aVelocityMultiplier;
    }

    public void goTo(double destinationX, double destinationY, double destinationHeading){
        boolean isActive = true;
        runtime.reset();
        while (isActive){
            boolean xActive = true;
            boolean yActive = true;
            boolean aActive = true;

            double frontLeftSpeed;
            double frontRightSpeed;
            double backLeftSpeed;
            double backRightSpeed;

            double vx = 0;
            double vy = 0;
            double va = 0;

            double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            gyroAngle = Rotation2d.fromDegrees(currentHeading);

            odometry.updateWithTime(runtime.time(), gyroAngle, wheelSpeeds);

            double currentX = odometry.getPoseMeters().getX();
            double currentY = odometry.getPoseMeters().getY();

            if (currentX + 0.05 < destinationX) {
                vx = 1;
            } else if (currentX - 0.05 > destinationX) {
                vx = -1;
            } else {
                vx = 0;
                xActive = false;
            }

            if (currentY + 0.05 < destinationY) {
                vy = 1;
            } else if (currentY - 0.05 > destinationY) {
                vy = -1;
            } else {
                vy = 0;
                yActive = false;
            }

            if (currentHeading + 0.05 < destinationHeading) {
                va = 1;
            } else if (currentHeading - 0.05 > destinationHeading) {
                va = -1;
            } else {
                va = 0;
                aActive = false;
            }

            vx *= vxMultiplier;
            vy *= vyMultiplier;
            va *= vaMultiplier;

            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, va, gyroAngle);
            wheelSpeeds = kinematics.toWheelSpeeds(speeds);

            frontLeftSpeed = wheelSpeeds.frontLeftMetersPerSecond;
            frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
            backLeftSpeed = wheelSpeeds.rearLeftMetersPerSecond;
            backRightSpeed = wheelSpeeds.rearRightMetersPerSecond;

            frontLeftMotor.setVelocity(frontLeftSpeed * METERS_TO_TICKS);
            frontRightMotor.setVelocity(frontRightSpeed * METERS_TO_TICKS);
            backLeftMotor.setVelocity(backLeftSpeed * METERS_TO_TICKS);
            backRightMotor.setVelocity(backRightSpeed * METERS_TO_TICKS);

            if (!xActive && !yActive && !aActive){
                isActive = false;
            }
        }
    }
}
