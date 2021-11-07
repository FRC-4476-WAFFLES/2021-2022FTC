package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

import java.util.ArrayList;

@Autonomous(name = "Gobuilda Auto Base")
public class GoBuildaAutoBase extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private MotorEx elevatorMotor;
    private MotorEx angleMotor;

    private MotorEx intakeMotor;

    private BNO055IMU imu;

    @Override
    public void runOpMode(){
        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        elevatorMotor = new MotorEx(hardwareMap, "Elevator");
        angleMotor = new MotorEx(hardwareMap, "Angle");

        intakeMotor = new MotorEx(hardwareMap, "Intake");

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

        GoBuildaChassisSubsystem mecanumNavigation = new GoBuildaChassisSubsystem(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor,imu);
        mecanumNavigation.initialize(0,0,0);
        mecanumNavigation.setMaxVelocity(1);
        mecanumNavigation.setTolerance(0.05,0.05,0.5);

        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevatorMotor, intakeMotor);
        elevatorSubsystem.initialize();

        waitForStart();

        mecanumNavigation.goTo(0,0,0);
        NavigationWaypoint p1 = new NavigationWaypoint(1,2);
        NavigationWaypoint p2 = new NavigationWaypoint(2,1,45);
        ArrayList<NavigationWaypoint> route1 = new ArrayList<NavigationWaypoint>();
        route1.add(p1);
        route1.add(p2);
        route1.add(new NavigationWaypoint(1,2));
        mecanumNavigation.goTo(route1);
        mecanumNavigation.goTo(5,5, 5);

        elevatorSubsystem.goTo(ElevatorSubsystem.Levels.INTAKE);
    }
}
