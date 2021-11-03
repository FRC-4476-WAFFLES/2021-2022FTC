package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.Hashtable;

public class ElevatorSubsystem extends SubsystemBase {
    private MotorEx elevatorMotor;
    private MotorEx angleMotor;

    private final double MM_PER_TICK = 0.12347;

    private final Hashtable<ElevatorLevels, Integer> levels;
    private final Hashtable<ElevatorLevels, Integer> angles;

    public ElevatorSubsystem(MotorEx elevatorMotor, Motor intakeMotor){
        this.elevatorMotor = elevatorMotor;
        this.levels = new Hashtable<ElevatorLevels, Integer>();
        this.angles = new Hashtable<ElevatorLevels, Integer>();

        levels.put(ElevatorLevels.INTAKE, 2);
        levels.put(ElevatorLevels.CAROUSEL, 2);
        levels.put(ElevatorLevels.L1, 0);
        levels.put(ElevatorLevels.L2, 0);
        levels.put(ElevatorLevels.L3, 0);

        angles.put(ElevatorLevels.INTAKE, 2);
        angles.put(ElevatorLevels.CAROUSEL, 2);
        angles.put(ElevatorLevels.L1, 0);
        angles.put(ElevatorLevels.L2, 0);
        angles.put(ElevatorLevels.L3, 0);
    }

    public void initialize(){
        elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        elevatorMotor.stopMotor();
        elevatorMotor.resetEncoder();
    }

    public void goToRawPosition(int target){
        elevatorMotor.setTargetPosition(target);
    }

    public void deploy(){

    }

    public void goTo(ElevatorLevels targetLevel){
        elevatorMotor.setTargetPosition(levels.get(targetLevel));
        angleMotor.setTargetPosition(angles.get(targetLevel));
    }

    public void periodic(){

    }

    public enum ElevatorLevels {
        INTAKE,
        CAROUSEL,
        L1,
        L2,
        L3
    }
}
