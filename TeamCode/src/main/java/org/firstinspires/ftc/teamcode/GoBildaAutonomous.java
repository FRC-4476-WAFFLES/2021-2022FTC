package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.FreightDetector;
import org.firstinspires.ftc.teamcode.Subsystems.DetectionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="GoBilda Autonomous")
public class GoBildaAutonomous extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private MotorEx elevatorMotor;
    private MotorEx intakeMotor;
    private MotorEx angleMotor;

    private ServoEx lockServo;

    private BNO055IMU imu;

    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private DetectionSubsystem detector;
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

        chassis = new DriveSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, imu);
        intake = new IntakeSubsystem(intakeMotor);
        elevator = new ElevatorSubsystem(elevatorMotor, angleMotor, lockServo);

        chassis.initialize(0, 0, 0);
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

        /*
        ElapsedTime tempTimer = new ElapsedTime();
        while (tempTimer.time() < 250) {
            targetLevel = pipeline.getTargetLevel();
            telemetry.addData("Target Level", targetLevel);
            telemetry.update();
        }
        */

        // targetLevel = pipeline.getTargetLevel();
        // targetLevel = ElevatorSubsystem.Levels.L2;
        // telemetry.addData("Target Level", targetLevel);

        // elevator.deploy();

        // telemetry.addLine("Deployed.");
        // telemetry.update();

        sleep(1500);

        // elevator.goTo(targetLevel);

        // telemetry.addLine("Elevator At Target Level");
        // telemetry.update();

        chassis.translate(.5, .5, false, telemetry);
        chassis.rotate(90, telemetry);

        /*
        while (opModeIsActive()) {
            telemetry.addData("Target Level", targetLevel);
            telemetry.addData("Target Motor Position", elevator.levels.get(targetLevel));
            telemetry.addData("Current Motor Position", elevatorMotor.getCurrentPosition());
            telemetry.update();
        }
        */
    }
}
