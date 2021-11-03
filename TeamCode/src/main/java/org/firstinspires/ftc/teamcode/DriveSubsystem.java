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

import java.util.ArrayList;

public class DriveSubsystem extends SubsystemBase {
    private final ElapsedTime runtime = new ElapsedTime();

    // Create the variables to hold the four motor objects
    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;

    private final BNO055IMU imu;

    private final double CPR = 2150.8; // Encoder ticks per wheel rotation
    private final double WHEEL_DIAMETER = 101.6; // Wheel diameter in millimeters
    private final double MM_PER_TICK = WHEEL_DIAMETER * 3.141592 / CPR; // Wheel distance traveled per encoder tick in millimeters
    private final double METERS_TO_TICKS = (1 / MM_PER_TICK) * 1000; // Convert meters per second to ticks per second

    private final double RADIANS_TO_DEGREES = 180 / 3.141592; // Convert radians per second to degrees per second

    // Create the objects needed for odometry and kinematics
    private MecanumDriveKinematics kinematics;
    private Rotation2d gyroAngle;
    private ChassisSpeeds chassisSpeeds;
    private MecanumDriveWheelSpeeds wheelSpeeds;
    private MecanumDriveOdometry odometry;

    private NavigationWaypoint currentWaypoint;
    private ArrayList<NavigationWaypoint> route;
    private boolean enRoute = false;
    private byte currentWaypointNumber = 0;

    private double hStart;

    // Set the default maximum chassis speeds in meters per second
    private double vMax = 1;

    // Set the default navigation tolerances in meters and degrees
    private double pxTol = 0.05;
    private double pyTol = 0.05;
    private double phTol = 0.5;

    public DriveSubsystem(MotorEx frontLeftMotor, MotorEx frontRightMotor, MotorEx backLeftMotor, MotorEx backRightMotor, BNO055IMU imu){
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.imu = imu;
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

        // Setup the odometry system
        // Set the locations of the wheels on the robot
        Translation2d frontLeftLocation = new Translation2d(0.167,0.195);
        Translation2d frontRightLocation = new Translation2d(0.167,-0.195);
        Translation2d backLeftLocation = new Translation2d(-0.167,0.195);
        Translation2d backRightLocation = new Translation2d(-0.167,-0.195);
        // Create a mecanum kinematics object from the wheel locations
        kinematics = new MecanumDriveKinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation);
        // Create an object to hold the angle of the robot as reported by the IMU
        gyroAngle = Rotation2d.fromDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + 180 - hStart);
        // Create a chassis speeds object relative to the field - format is (desired speed relative to one wall, desired speed relative to second wall, desired rotation speed in radians, current angle)
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, gyroAngle);
        // Get wheel speeds from the chassis speed
        wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        // Create an odometry object - format is (kinematics object, gyro heading, start position on field)
        odometry = new MecanumDriveOdometry(kinematics, gyroAngle, new Pose2d(xStart, yStart, new Rotation2d(hStart)));
    }

    public void periodic(){
        updateOdometry();

        if (enRoute){
            double frontLeftSpeed;
            double frontRightSpeed;
            double backLeftSpeed;
            double backRightSpeed;

            ArrayList<Double> multipliers = getVelocityMultipliers(getDistances(currentWaypoint));

            double vx = multipliers.get(0) * vMax;
            double vy = multipliers.get(1) * vMax;
            double vh = multipliers.get(2) * vMax;

            gyroAngle = Rotation2d.fromDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + 180 - hStart);

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vh, gyroAngle);
            wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

            frontLeftSpeed = wheelSpeeds.frontLeftMetersPerSecond;
            frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
            backLeftSpeed = wheelSpeeds.rearLeftMetersPerSecond;
            backRightSpeed = wheelSpeeds.rearRightMetersPerSecond;

            frontLeftMotor.setVelocity(frontLeftSpeed * METERS_TO_TICKS);
            frontRightMotor.setVelocity(frontRightSpeed * METERS_TO_TICKS);
            backLeftMotor.setVelocity(backLeftSpeed * METERS_TO_TICKS);
            backRightMotor.setVelocity(backRightSpeed * METERS_TO_TICKS);

            if (vx == 0 && vy == 0 && vh == 0){
                nextWaypoint();
            }
        }
    }

    /**
     * Set the maximum speed of the robot.
     * @param translate The max speed of the robot, in meters per second
     */

    public void setMaxVelocity(double translate){
        vMax = translate;
    }

    /**
     * Set the destination tolerance of the robot.
     * Units are meters and degrees.
     * @param xTolerance Tolerance along x-axis
     * @param yTolerance Tolerance along y-axis
     * @param hTolerance Tolerance along heading
     */

    public void setTolerance(double xTolerance, double yTolerance, double hTolerance){
        pxTol = xTolerance;
        pyTol = yTolerance;
        phTol = hTolerance;
    }

    /**
     * Drive the robot to a specified destination.
     * @param destinationX X Co-ordinate of the target
     * @param destinationY Y Co-ordinate of the target
     * @param destinationH Heading of the target
     */
    public void goTo(double destinationX, double destinationY, double destinationH) {
        if (!enRoute){
            currentWaypoint = new NavigationWaypoint(destinationX, destinationY, destinationH);
        }
    }

    /**
     * Drive the robot to a specified destination
     * @param destination NavigationWaypoint destination
     */
    public void goTo(NavigationWaypoint destination){
        if (!enRoute){
            currentWaypoint = destination;
        }
    }

    /**
     * Drive the robot along a path
     * @param route List of waypoints for the robot to follow
     */
    public void goTo(ArrayList<NavigationWaypoint> route){
        if (!enRoute){
            this.route = route;
        }
    }

    private void driveTo(NavigationWaypoint waypoint){
        runtime.reset();
        boolean isFinished = false;

        while (!isFinished){
            double frontLeftSpeed;
            double frontRightSpeed;
            double backLeftSpeed;
            double backRightSpeed;

            updateOdometry();

            ArrayList<Double> multipliers = getVelocityMultipliers(getDistances(waypoint));

            double vx = multipliers.get(0) * vMax;
            double vy = multipliers.get(1) * vMax;
            double vh = multipliers.get(2) * vMax;

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vh, gyroAngle);
            wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

            frontLeftSpeed = wheelSpeeds.frontLeftMetersPerSecond;
            frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
            backLeftSpeed = wheelSpeeds.rearLeftMetersPerSecond;
            backRightSpeed = wheelSpeeds.rearRightMetersPerSecond;

            frontLeftMotor.setVelocity(frontLeftSpeed * METERS_TO_TICKS);
            frontRightMotor.setVelocity(frontRightSpeed * METERS_TO_TICKS);
            backLeftMotor.setVelocity(backLeftSpeed * METERS_TO_TICKS);
            backRightMotor.setVelocity(backRightSpeed * METERS_TO_TICKS);

            if (vx == 0 && vy == 0 && vh == 0){
                isFinished = true;
            }
        }
    }

    private ArrayList<Double> getDistances(NavigationWaypoint waypoint){
        ArrayList<Double> distances = new ArrayList<>();

        distances.add((Math.abs(waypoint.x) < pxTol) ? 0 : ((waypoint.fieldRelative) ? waypoint.x - odometry.getPoseMeters().getX() * 3 : waypoint.x));
        distances.add((Math.abs(waypoint.y) < pyTol) ? 0 : ((waypoint.fieldRelative) ? waypoint.y - odometry.getPoseMeters().getY() * 3 : waypoint.y));
        distances.add((Math.abs(waypoint.h) < phTol) ? 0 : ((waypoint.fieldRelative) ? waypoint.h - (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + 180 - hStart) : waypoint.h));

        return distances;
    }

    private ArrayList<Double> getVelocityMultipliers(ArrayList<Double> distances){
        ArrayList<Double> multipliers = new ArrayList<>();

        double dx = distances.get(0);
        double dy = distances.get(1);
        double dh = distances.get(2);

        boolean dxIsNegative = dx < 0;
        boolean dyIsNegative = dy < 0;
        boolean dhIsNegative = dh < 0;

        double vxMultiplier;
        double vyMultiplier;
        double vhMultiplier;

        dh /= RADIANS_TO_DEGREES;

        dx = Math.abs(dx);
        dy = Math.abs(dy);
        dh = Math.abs(dh);

        if (dx >= dy){
            if (dx >= dh){
                vxMultiplier = 1;
                vyMultiplier = dy / dx;
                vhMultiplier = dh / dx;
            } else {
                vxMultiplier = dx / dh;
                vyMultiplier = dy / dh;
                vhMultiplier = 1;
            }
        } else if (dy >= dh){
            vxMultiplier = dx / dy;
            vyMultiplier = 1;
            vhMultiplier = dh / dy;
        } else {
            vxMultiplier = dx / dh;
            vyMultiplier = dy / dh;
            vhMultiplier = 1;
        }

        multipliers.add((dxIsNegative) ? -vxMultiplier : vxMultiplier);
        multipliers.add((dyIsNegative) ? -vyMultiplier : vyMultiplier);
        multipliers.add((dhIsNegative) ? -vhMultiplier : vhMultiplier);

        return multipliers;
    }

    private void updateOdometry(){
        gyroAngle = Rotation2d.fromDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + 180 - hStart);

        wheelSpeeds = new MecanumDriveWheelSpeeds(
                frontLeftMotor.getVelocity() / METERS_TO_TICKS,
                frontRightMotor.getVelocity() / METERS_TO_TICKS,
                backLeftMotor.getVelocity() / METERS_TO_TICKS,
                backRightMotor.getVelocity() / METERS_TO_TICKS);

        odometry.updateWithTime(runtime.time(), gyroAngle, wheelSpeeds);
    }

    private void nextWaypoint(){
        if (currentWaypointNumber < route.size()){
            currentWaypointNumber++;
            currentWaypoint = route.get(currentWaypointNumber);
        } else {
            enRoute = false;
        }
    }
}
