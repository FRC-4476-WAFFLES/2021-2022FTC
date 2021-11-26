package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@Autonomous(name="Translation Test")
public class TranslationTest extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private GyroEx gyro;

    private DriveSubsystem chassis;

    @Override
    public void runOpMode(){
        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        gyro = new RevIMU(hardwareMap, "imu");

        chassis = new DriveSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, gyro);

        chassis.initialize(0, 0, 0);

        chassis.setMaxVelocity(0.2);

        telemetry.addLine("Waiting for start");

        telemetry.update();

        waitForStart();

        chassis.translate(2, -1, 0, true, telemetry);

        sleep(1000);

        chassis.translate(-1, 3, 0, true, telemetry);

        chassis.translate(1, -1, 0, false, telemetry);

        //chassis.translate(0, 2, false, telemetry);
    }
}
