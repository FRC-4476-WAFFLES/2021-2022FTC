package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@TeleOp(name="Levels test")
public class LevelsTest extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private MotorEx elevatorMotor;
    private MotorEx intakeMotor;
    private MotorEx angleMotor;

    private ServoEx lockServo;

    private GyroEx gyro;

    private DigitalChannel elevatorLimit;

    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private DriveSubsystem chassis;

    private GamepadEx operatorJoystick;

    ElevatorSubsystem.Levels targetLevel;

    @Override
    public void runOpMode() {
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

        operatorJoystick = new GamepadEx(gamepad2);

        chassis = new DriveSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, gyro);
        intake = new IntakeSubsystem(intakeMotor);
        elevator = new ElevatorSubsystem(elevatorMotor, angleMotor, lockServo, elevatorLimit);

        chassis.initialize(1, 0.22, 0);
        chassis.setMaxVelocity(0.2, 0.2);
        chassis.setTolerance(0.02);

        elevator.initialize();

        waitForStart();

        while (opModeIsActive()){
            if (operatorJoystick.getButton(GamepadKeys.Button.A)){
                elevator.goTo(ElevatorSubsystem.Levels.INTAKE);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.B)){
                elevator.goTo(ElevatorSubsystem.Levels.L1);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.X)){
                elevator.goTo(ElevatorSubsystem.Levels.L2);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.Y)){
                elevator.goTo(ElevatorSubsystem.Levels.CAROUSEL);
            } else if (operatorJoystick.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                elevator.goTo(ElevatorSubsystem.Levels.L3);
            }
        }
    }
}
