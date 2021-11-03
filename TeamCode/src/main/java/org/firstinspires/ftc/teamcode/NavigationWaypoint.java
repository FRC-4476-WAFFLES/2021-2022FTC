package org.firstinspires.ftc.teamcode;

public class NavigationWaypoint {
    double x;
    double y;
    double h;
    boolean fieldRelative;
    ElevatorLevels elevatorLevel;

    NavigationWaypoint(){
        this.x = 0;
        this.y = 0;
        this.h = 0;
        this.fieldRelative = true;
        this.elevatorLevel = ElevatorLevels.CONSTANT;
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

    public NavigationWaypoint(double x, double y, ElevatorLevels elevatorLevel){
        this.x = x;
        this.y = y;
        this.elevatorLevel = elevatorLevel;
    }

    public NavigationWaypoint(double x, double y, double h, ElevatorLevels elevatorLevel){
        this.x = x;
        this.y = y;
        this.h = h;
        this.elevatorLevel = elevatorLevel;
    }

    public NavigationWaypoint(double x, double y, boolean fieldRelative, ElevatorLevels elevatorLevel){
        this.x = x;
        this.y = y;
        this.fieldRelative = fieldRelative;
        this.elevatorLevel = elevatorLevel;
    }

    public NavigationWaypoint(double x, double y, double h, boolean fieldRelative, ElevatorLevels elevatorLevel){
        this.x = x;
        this.y = y;
        this.h = h;
        this.fieldRelative = fieldRelative;
        this.elevatorLevel = elevatorLevel;
    }
}