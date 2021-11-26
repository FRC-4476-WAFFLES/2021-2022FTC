package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NavigationWaypoint;

public class DriveSubsystem extends SubsystemBase {
    private final ElapsedTime runtime = new ElapsedTime();

    // Create the variables to hold the four motor objects
    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;

    private final GyroEx gyro;

    private final double CPR = 2150.8; // Encoder ticks per wheel rotation
    private final double WHEEL_DIAMETER = 101.6; // Wheel diameter in millimeters
    private final double MM_PER_TICK = (WHEEL_DIAMETER * 3.141592 / CPR) * 3; // Wheel distance traveled per encoder tick in millimeters
    private final double METERS_TO_TICKS = (1 / MM_PER_TICK) * 1000; // Convert meters per second to ticks per second

    // Create the objects needed for odometry and kinematics
    private MecanumDriveKinematics kinematics;
    private Rotation2d gyroAngle;
    private ChassisSpeeds chassisSpeeds;
    private MecanumDriveWheelSpeeds wheelSpeeds;
    private MecanumDriveOdometry odometry;

    private NavigationWaypoint currentWaypoint;
    private boolean isActive = false;

    private double hStart;

    // Set the default maximum chassis speeds in meters per second
    private double vMax = 1;

    // Set the default navigation tolerances in meters and radians
    private double pxTol = 0.05;
    private double pyTol = 0.05;
    private double phTol = 0.05;

    public DriveSubsystem(MotorEx frontLeftMotor, MotorEx frontRightMotor, MotorEx backLeftMotor, MotorEx backRightMotor, GyroEx gyro){
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.gyro = gyro;
    }

    /**
     * Initialize the subsystem
     * Units are meters and degrees
     *
     * @param xStart X location on field at start
     * @param yStart Y location on field at start
     * @param hStart Robot heading on field at start
     */

    public void initialize(double xStart, double yStart, double hStart){
        this.hStart = hStart;
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

        // Set how far the wheels move in millimeters per encoder tick
        frontLeftMotor.setDistancePerPulse(MM_PER_TICK);
        frontRightMotor.setDistancePerPulse(MM_PER_TICK);
        backLeftMotor.setDistancePerPulse(MM_PER_TICK);
        backRightMotor.setDistancePerPulse(MM_PER_TICK);

        gyro.init();

        // Setup the odometry system
        // Set the locations of the wheels on the robot
        Translation2d frontLeftLocation = new Translation2d(0.167,0.195);
        Translation2d frontRightLocation = new Translation2d(0.167,-0.195);
        Translation2d backLeftLocation = new Translation2d(-0.167,0.195);
        Translation2d backRightLocation = new Translation2d(-0.167,-0.195);
        // Create a mecanum kinematics object from the wheel locations
        kinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        // Create an object to hold the angle of the robot as reported by the IMU
        gyroAngle = gyro.getRotation2d();
        // Create a chassis speeds object relative to the field - format is (desired speed relative to one wall, desired speed relative to second wall, desired rotation speed in radians, current angle)
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, gyroAngle);
        // Get wheel speeds from the chassis speed
        wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        // Create an odometry object - format is (kinematics object, gyro heading, start position on field)
        odometry = new MecanumDriveOdometry(kinematics, gyroAngle, new Pose2d(xStart, yStart, new Rotation2d(hStart)));
    }

    public void periodic(){
    }

    /**
     * Set the maximum speed of the robot.
     * @param translate The max speed of the robot, in meters per second
     */

    public void setMaxVelocity(double translate){
        vMax = translate;
    }

    public void translate(double x, double y, double h, boolean fieldRelative, Telemetry telemetry){
        updateOdometry();

        Rotation2d heading = getHeadingAsRotation2d();

        final Pose2d destination;

        if (fieldRelative) {
            destination = new Pose2d(x, y, Rotation2d.fromDegrees(h));
        } else {
            destination = new Pose2d(x + odometry.getPoseMeters().getX(), y + odometry.getPoseMeters().getY(), heading.plus(Rotation2d.fromDegrees(h)));
        }

        while (true){
            double vxMultiplier;
            double vyMultiplier;
            double vhMultiplier;

            updateOdometry();

            heading = getHeadingAsRotation2d();

            ChassisSpeeds speeds;

            if (Math.abs(odometry.getPoseMeters().getX() - destination.getX()) < pxTol){
                vxMultiplier = 0;
            } else if (odometry.getPoseMeters().getX() < destination.getX()){
                vxMultiplier = 1;
            } else {
                vxMultiplier = -1;
            }

            if (Math.abs(odometry.getPoseMeters().getY() - destination.getY()) < pyTol){
                vyMultiplier = 0;
            } else if (odometry.getPoseMeters().getY() < destination.getY()){
                vyMultiplier = 1;
            } else {
                vyMultiplier = -1;
            }

            if (Math.abs(heading.getRadians() - destination.getHeading()) < phTol){
                vhMultiplier = 0;
            } else if (heading.getRadians() < destination.getHeading()){
                vhMultiplier = 1;
            } else {
                vhMultiplier = -1;
            }

            if (vxMultiplier == 0 && vyMultiplier == 0 && vhMultiplier == 0){
                break;
            }

            if (fieldRelative){
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMultiplier * vMax, vyMultiplier * vMax, vhMultiplier * vMax, heading);
            } else {
                speeds = new ChassisSpeeds(vxMultiplier * vMax, vyMultiplier * vMax, vhMultiplier * vMax);
            }

            setMotors(kinematics.toWheelSpeeds(speeds));

            telemetry.addData("Field Relative", fieldRelative);
            telemetry.addData("Location", odometry.getPoseMeters());
            telemetry.addLine();
            telemetry.addData("Destination", destination);
            telemetry.addLine();
            telemetry.addData("x Multiplier", vxMultiplier);
            telemetry.addData("y Multiplier", vyMultiplier);
            telemetry.addData("h Multiplier", vhMultiplier);
            telemetry.update();
        }

        stop();
    }

    public void stop(){
        frontLeftMotor.setVelocity(0);
        frontRightMotor.setVelocity(0);
        backLeftMotor.setVelocity(0);
        backRightMotor.setVelocity(0);
        wheelSpeeds = new MecanumDriveWheelSpeeds(0, 0, 0, 0);
        frontLeftMotor.disable();
        frontRightMotor.disable();
        backLeftMotor.disable();
        backRightMotor.disable();
    }

    private void updateOdometry(){
        odometry.updateWithTime(runtime.time(),
                getHeadingAsRotation2d(),
                new MecanumDriveWheelSpeeds(
                frontLeftMotor.getVelocity() / METERS_TO_TICKS,
                frontRightMotor.getVelocity() / METERS_TO_TICKS,
                backLeftMotor.getVelocity() / METERS_TO_TICKS,
                backRightMotor.getVelocity() / METERS_TO_TICKS));
    }

    private Rotation2d getHeadingAsRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    private double getHeading(){
        double heading = gyro.getAbsoluteHeading() - hStart;
        if (heading < -180) {
            heading += 360;
        } else if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    private void setMotors(MecanumDriveWheelSpeeds wheelSpeeds){
        frontLeftMotor.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * METERS_TO_TICKS);
        frontRightMotor.setVelocity(wheelSpeeds.frontRightMetersPerSecond * METERS_TO_TICKS);
        backLeftMotor.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * METERS_TO_TICKS);
        backRightMotor.setVelocity(wheelSpeeds.rearRightMetersPerSecond * METERS_TO_TICKS);
    }
}
