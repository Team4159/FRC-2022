package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboard {
    private ShuffleboardTab driveTeam = Shuffleboard.getTab("Drive Team");
    private ShuffleboardTab electrical = Shuffleboard.getTab("Electrical");
    private ShuffleboardTab testing = Shuffleboard.getTab("Testing");

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
