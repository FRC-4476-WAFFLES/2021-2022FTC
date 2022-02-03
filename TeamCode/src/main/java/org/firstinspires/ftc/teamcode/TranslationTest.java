package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

@Disabled
@Autonomous(name="Translation Test")
public class TranslationTest extends LinearOpMode {
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

    private DriveSubsystem chassis;
    private ElevatorSubsystem elevator;

    @Override
    public void runOpMode(){
        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        elevatorMotor = new MotorEx(hardwareMap, "intakeRaise");
        angleMotor = new MotorEx(hardwareMap, "Elevator");
        intakeMotor = new MotorEx(hardwareMap, "intakeSpin");

        lockServo = new SimpleServo(hardwareMap, "Lock", 0, 90, AngleUnit.DEGREES);

        gyro = new RevIMU(hardwareMap, "imu");

        elevatorLimit = hardwareMap.get(DigitalChannel.class, "intakeRaiseLimit");

        chassis = new DriveSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, gyro);
        elevator = new ElevatorSubsystem(elevatorMotor, angleMotor, lockServo, elevatorLimit);

        chassis.initialize(1, 0.22, 0);
        chassis.setMaxVelocity(0.4,0.4);
        chassis.setTolerance(0.02);

        elevator.initialize();

        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        elevator.goTo(ElevatorSubsystem.Levels.L3);

        chassis.translate(1.5, 0.58, 0);

        intakeMotor.set(-1);

        sleep(800);

        intakeMotor.set(0);

        elevator.goTo(ElevatorSubsystem.Levels.CAROUSEL);

        chassis.setTolerance(0.01);
        chassis.translate(0.52, 0.24, 90, 4);

        intakeMotor.set(-1);

        sleep(4000);

        intakeMotor.set(0);

        chassis.setMaxVelocity(0.4, 0.6);
        chassis.translate(1.5, 0.24, -90,5);
        chassis.setMaxVelocity(0.4, 0.4);
        chassis.translate(2, 0.15, -90, 2);
        chassis.translate(3, 0.15, -90, 2);

        elevator.goTo(ElevatorSubsystem.Levels.L1);
    }
}
