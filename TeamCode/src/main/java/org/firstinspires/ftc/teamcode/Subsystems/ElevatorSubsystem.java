package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.Hashtable;

public class ElevatorSubsystem extends SubsystemBase {
    private MotorEx elevatorMotor;
    private MotorEx angleMotor;

    private final double MM_PER_TICK = 0.12347;

    private final Hashtable<Levels, Integer> levels;

    private boolean deploy = false;
    private boolean deployed = false;

    public ElevatorSubsystem(MotorEx elevatorMotor, MotorEx angleMotor){
        this.elevatorMotor = elevatorMotor;
        this.angleMotor = angleMotor;
        this.levels = new Hashtable<>();

        levels.put(Levels.INTAKE, 2);
        levels.put(Levels.CAROUSEL, 2);
        levels.put(Levels.L1, 0);
        levels.put(Levels.L2, 0);
        levels.put(Levels.L3, 0);
    }

    public void initialize(){
        elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        elevatorMotor.stopMotor();
        elevatorMotor.resetEncoder();

        angleMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void goToRawPosition(int target){
        elevatorMotor.setTargetPosition(target);
    }

    public void deploy(){
        deploy = true;
    }

    public void goTo(Levels targetLevel){
        elevatorMotor.setTargetPosition(levels.get(targetLevel));
    }

    public void periodic(){
        if(deploy && !deployed){
            if(angleMotor.getVelocity() < 1000 && angleMotor.getCurrentPosition() > 100){
                angleMotor.set(0);
                deployed = true;
            } else {
                angleMotor.set(0.2);
            }
        }
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
