package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {


    public Dashboard() {
        display();
    }

    public void display() {
        intakeInfo();
        feederInfo();
        climberInfo();
        shooterInfo();
        visionInfo();
    }

    public void intakeInfo() {
        SmartDashboard.putBoolean("String", true); //Example
    }

    public void feederInfo() {

    }


    public void climberInfo() {

    }

    public void shooterInfo() {

    }

    public void visionInfo() {

    }



}
