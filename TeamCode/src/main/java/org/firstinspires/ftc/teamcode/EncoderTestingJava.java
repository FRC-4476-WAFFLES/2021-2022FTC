package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderTestingJava extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private BNO055IMU imu;
    @Override
    public void runOpMode() {
        // Assign variables to corresponding hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor = hardwareMap.get(DcMotor.class, "BR");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Initialization Code
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for game to start
        waitForStart();
        runtime.reset();

        // Drive 3 inches at 1 inch per second with a timeout of 5 seconds
        encoderDrive(1, 3, 5.0);

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
    private void encoderDrive(double speed, double inches, double timeoutS) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // Determine new target position, and pass to motor controller
        int targetPosition = (int)(inches * COUNTS_PER_INCH);
        int FL_Target = targetPosition + frontLeftMotor.getCurrentPosition();
        int FR_Target = targetPosition + frontRightMotor.getCurrentPosition();
        int BL_Target = targetPosition + backLeftMotor.getCurrentPosition();
        int BR_Target = targetPosition + backLeftMotor.getCurrentPosition();
        frontLeftMotor.setTargetPosition(FL_Target);
        frontRightMotor.setTargetPosition(FR_Target);
        backLeftMotor.setTargetPosition(BL_Target);
        backRightMotor.setTargetPosition(BR_Target);
        // Turn On RUN_TO_POSITION
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        driveTime.reset();
        speed = Math.abs(speed);
        setPower(speed, speed, speed, speed);
        while (opModeIsActive() &&
                (driveTime.seconds() < timeoutS) &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Target Position",  "FL: %7d, FR: %7d, BL: %7d, BR: %7d",
                    FL_Target,  FR_Target, BL_Target, BR_Target);
            telemetry.addData("Current Position",  "FL: %7d, FR: %7d, BL: %7d, BR: %7d",
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition());
            telemetry.update();
        }
        // Stop all motion
        setPower(0, 0, 0, 0);
        // Turn off RUN_TO_POSITION
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setPower(double FL, double FR, double BL, double BR) {
        frontLeftMotor.setPower(FL);
        frontRightMotor.setPower(FR);
        backLeftMotor.setPower(BL);
        backRightMotor.setPower(BR);
    }
}