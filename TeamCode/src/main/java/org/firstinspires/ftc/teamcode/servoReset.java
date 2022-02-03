package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@Autonomous(name = "turn servo to position 0 degrees")
public class servoReset extends LinearOpMode {
    ServoEx asdf;
    @Override
    public void runOpMode(){
        asdf = new SimpleServo(hardwareMap, "Lock", 0, 90, AngleUnit.DEGREES);

        waitForStart();

        asdf.turnToAngle(0);
    }
}
