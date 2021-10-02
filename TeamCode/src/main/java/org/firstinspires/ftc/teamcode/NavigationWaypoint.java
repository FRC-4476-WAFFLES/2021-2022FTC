package org.firstinspires.ftc.teamcode;

public class NavigationWaypoint {
    double x;
    double y;
    double h;

    NavigationWaypoint(){
        this.x = 0;
        this.y = 0;
        this.h = 0;
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
}
