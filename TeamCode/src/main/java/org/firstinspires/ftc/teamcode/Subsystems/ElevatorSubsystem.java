package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Hashtable;

public class ElevatorSubsystem extends SubsystemBase {
    private final MotorEx elevatorMotor;
    private final MotorEx angleMotor;

    private final ServoEx lockServo;

    private final DigitalChannel elevatorLimit;

    private final double TICKS_PER_ROTATION = 1440;
    private final double MM_PER_TICK = (-30 * Math.PI) / (TICKS_PER_ROTATION);

    public final Hashtable<Levels, Integer> levels;

    public Levels targetLevel;

    public ElevatorSubsystem(MotorEx elevatorMotor, MotorEx angleMotor, ServoEx lockServo, DigitalChannel elevatorLimit){
        this.elevatorMotor = elevatorMotor;
        this.angleMotor = angleMotor;
        this.lockServo = lockServo;
        this.elevatorLimit = elevatorLimit;
        this.levels = new Hashtable<>();

        levels.put(Levels.INTAKE, 0);
        levels.put(Levels.CAROUSEL, (int) (250 / MM_PER_TICK));
        levels.put(Levels.L1, (int) (90 / MM_PER_TICK));
        levels.put(Levels.L2, (int) (230 / MM_PER_TICK));
        levels.put(Levels.L3, (int) (370 / MM_PER_TICK));
    }

    public void initialize(){
        elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        angleMotor.setRunMode(Motor.RunMode.PositionControl);

        elevatorMotor.resetEncoder();
        angleMotor.resetEncoder();

        // set the tolerance
        elevatorMotor.setPositionTolerance(15);
        angleMotor.setPositionTolerance(15);

        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void goToRawPosition(int target, double power){
        elevatorMotor.setTargetPosition(target);
        while (!elevatorMotor.atTargetPosition()) {
            if (!elevatorLimit.getState() && elevatorMotor.getCurrentPosition() > target){
                break;
            }
            elevatorMotor.set(power);
        }
        elevatorMotor.set(0);
    }

    public void deploy(){
        angleMotor.resetEncoder();
        elevatorMotor.resetEncoder();

        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        elevatorMotor.setTargetPosition(1440 * 2);

        while (!elevatorMotor.atTargetPosition()) elevatorMotor.set(0.6);

        angleMotor.setTargetPosition((int) (1440 * -0.375));
        elevatorMotor.setTargetPosition((int) (1440 * 5.5));

        boolean angleComplete = false;
        boolean elevatorComplete = false;

        do {
            if (angleMotor.atTargetPosition()) {
                angleMotor.set(0);
                angleComplete = true;
            } else {
                angleMotor.set(0.6);
            }

            if (elevatorMotor.atTargetPosition()) {
                elevatorMotor.set(0);
                elevatorComplete = true;
            } else {
                elevatorMotor.set(1);
            }

        } while (!elevatorComplete && !angleComplete);

        angleMotor.disable();
        elevatorMotor.set(0);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time() < 0.75) { }

        lockServo.turnToAngle(90, AngleUnit.DEGREES);

        timer.reset();
        while (timer.time() < 0.5) { }

        elevatorMotor.setTargetPosition(1440 * 5);
        while (!elevatorMotor.atTargetPosition()) elevatorMotor.set(0.6);
        elevatorMotor.set(0);
        elevatorMotor.resetEncoder();
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void goTo(Levels targetLevel){
        goToRawPosition((targetLevel == Levels.CONSTANT) ? elevatorMotor.getCurrentPosition() : levels.get(targetLevel), 0.6);
    }

    public void goToTeleOp(Levels targetLevel){
        this.targetLevel = targetLevel;
    }

    public void update(){
        elevatorMotor.setTargetPosition(levels.get(targetLevel));
        if (!elevatorMotor.atTargetPosition() &&
                elevatorLimit.getState() &&
                elevatorMotor.getCurrentPosition() < levels.get(targetLevel)) {

            elevatorMotor.set(0.6);
        }
        elevatorMotor.set(0);
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
