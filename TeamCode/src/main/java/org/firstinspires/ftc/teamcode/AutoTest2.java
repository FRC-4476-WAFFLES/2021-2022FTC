package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.FreightDetector;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto Test 2")
public class AutoTest2 extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private MotorEx elevatorMotor;
    private MotorEx intakeMotor;
    private MotorEx angleMotor;

    private ServoEx lockServo;

    private GyroEx gyro;

    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private DriveSubsystem chassis;

    private int cameraMonitorViewId;
    private OpenCvCamera webcam;

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

        chassis = new DriveSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, gyro);
        intake = new IntakeSubsystem(intakeMotor);
        elevator = new ElevatorSubsystem(elevatorMotor, angleMotor, lockServo);

        chassis.initialize(1, 0.22, 0);
        chassis.setMaxVelocity(0.2);
        chassis.setTolerance(0.02);

        elevator.initialize();

        // Get webcam object from hardwareMap
        cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set pipeline to Freight Detector
        FreightDetector pipeline = new FreightDetector(telemetry);
        webcam.setPipeline(pipeline);

        // Activate camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming from the camera
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Opened webcam.");
            }

            @Override
            public void onError(int errorCode) {
                // Camera fails to be opened
                telemetry.addData("Failed to open webcam. Error code:", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        while (!isStarted()) {
            targetLevel = pipeline.getTargetLevel();
            telemetry.addData("Target Level", targetLevel);
            telemetry.update();
        }

        waitForStart();

        webcam.stopStreaming();

        elevator.deploy();

        sleep(500);

        if (targetLevel == ElevatorSubsystem.Levels.L3){
            elevator.goTo(ElevatorSubsystem.Levels.L2);
        } else {
            elevator.goTo(targetLevel);
        }

        chassis.translate(1.5, 0.58, 0, true, telemetry);

        intakeMotor.set(-1);

        sleep(800);

        intakeMotor.set(0);

        elevator.goTo(ElevatorSubsystem.Levels.CAROUSEL);

        chassis.setTolerance(0.01);

        chassis.translate(0.5, 0.24, 90, 8);

        intakeMotor.set(-1);

        sleep(4000);

        intakeMotor.set(0);

        chassis.translate(0.52, 0.8, 90, true, telemetry);
    }
}
