package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.Hashtable;

public class IntakeSubsystem extends SubsystemBase {
    private MotorEx intakeMotor;
    private Hashtable<Modes, Double> modes;
    public IntakeSubsystem(MotorEx intakeMotor){
        this.intakeMotor = intakeMotor;
        this.modes = new Hashtable<>();
        modes.put(Modes.INTAKE, (double) 1);
        modes.put(Modes.OUTTAKE, (double) -1);
        modes.put(Modes.OFF, (double) 0);
    }

    public enum Modes {
        INTAKE,
        OUTTAKE,
        OFF
    }

    public void operate(Modes operationMode){
        intakeMotor.set(modes.get(operationMode));

    }

    public void stop(){
        intakeMotor.stopMotor();
    }
}
