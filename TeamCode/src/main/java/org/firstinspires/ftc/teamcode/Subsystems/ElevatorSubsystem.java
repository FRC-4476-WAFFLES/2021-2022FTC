package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Hashtable;

public class ElevatorSubsystem extends SubsystemBase {
    private MotorEx elevatorMotor;
    private MotorEx angleMotor;

    private ServoEx lockServo;

    private final double TICKS_PER_ROTATION = 1440;
    private final double MM_PER_TICK = (-50 * Math.PI) / TICKS_PER_ROTATION;

    public final Hashtable<Levels, Integer> levels;

    private boolean deploy = false;
    private boolean deployed = false;

    public ElevatorSubsystem(MotorEx elevatorMotor, MotorEx angleMotor, ServoEx lockServo){
        this.elevatorMotor = elevatorMotor;
        this.angleMotor = angleMotor;
        this.lockServo = lockServo;
        this.levels = new Hashtable<>();

        levels.put(Levels.INTAKE, 0);
        levels.put(Levels.CAROUSEL, (int) (330/MM_PER_TICK));
        levels.put(Levels.L1, (int) (150/MM_PER_TICK));
        levels.put(Levels.L2, (int) (280/MM_PER_TICK));
        levels.put(Levels.L3, (int) (320/MM_PER_TICK));
    }

    public void initialize(){
        elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

        elevatorMotor.resetEncoder();
        angleMotor.resetEncoder();

        // set the tolerance
        elevatorMotor.setPositionTolerance(15);
        angleMotor.setPositionTolerance(15);
    }

    public void goToRawPosition(int target, double power){
        elevatorMotor.setTargetPosition(target);
        while (!elevatorMotor.atTargetPosition()) elevatorMotor.set(power);
        elevatorMotor.set(0);
    }

    public void deploy(){
        goToRawPosition((int) (1440 * 2), 0.6);

        angleMotor.setTargetPosition((int) (1440 * -0.375));
        while (!angleMotor.atTargetPosition()) angleMotor.set(0.6);
        angleMotor.set(0);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time() < 0.250) { }

        lockServo.turnToAngle(90, AngleUnit.DEGREES);

        elevatorMotor.resetEncoder();
    }

    public void goTo(Levels targetLevel){
        goToRawPosition((targetLevel == Levels.CONSTANT) ? elevatorMotor.getCurrentPosition() : levels.get(targetLevel), 0.6);
    }

    public void periodic(){

    }

    public void stop(){
        elevatorMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public enum Levels {
        CONSTANT,
        INTAKE,
        CAROUSEL,
        L1,
        L2,
        L3
    }
}
