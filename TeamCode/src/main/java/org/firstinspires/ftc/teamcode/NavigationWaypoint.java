package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class NavigationWaypoint {
    public double x;
    public double y;
    public double h;
    public boolean fieldRelative;
    public ElevatorSubsystem.Levels elevatorLevel;
    public IntakeSubsystem.Modes intakeMode;

    NavigationWaypoint(){
        this.x = 0;
        this.y = 0;
        this.h = 0;
        this.fieldRelative = true;
        this.elevatorLevel = ElevatorSubsystem.Levels.CONSTANT;
        this.intakeMode = IntakeSubsystem.Modes.OFF;
    }

    public NavigationWaypoint(double x, double y){
        this.x = x;
        this.y = y;
    }

    public NavigationWaypoint(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public NavigationWaypoint(double x, double y, boolean fieldRelative){
        this.x = x;
        this.y = y;
        this.fieldRelative = fieldRelative;
    }

    public NavigationWaypoint(double x, double y, double h, boolean fieldRelative){
        this.x = x;
        this.y = y;
        this.h = h;
        this.fieldRelative = fieldRelative;
    }

    public NavigationWaypoint(double x, double y, ElevatorSubsystem.Levels elevatorLevel){
        this.x = x;
        this.y = y;
        this.elevatorLevel = elevatorLevel;
    }

    public NavigationWaypoint(double x, double y, double h, ElevatorSubsystem.Levels elevatorLevel){
        this.x = x;
        this.y = y;
        this.h = h;
        this.elevatorLevel = elevatorLevel;
    }

    public NavigationWaypoint(double x, double y, boolean fieldRelative, ElevatorSubsystem.Levels elevatorLevel){
        this.x = x;
        this.y = y;
        this.fieldRelative = fieldRelative;
        this.elevatorLevel = elevatorLevel;
    }

    public NavigationWaypoint(double x, double y, double h, boolean fieldRelative, ElevatorSubsystem.Levels elevatorLevel){
        this.x = x;
        this.y = y;
        this.h = h;
        this.fieldRelative = fieldRelative;
        this.elevatorLevel = elevatorLevel;
    }
}