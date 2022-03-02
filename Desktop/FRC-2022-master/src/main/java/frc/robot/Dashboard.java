package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
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
        ShuffleboardLayout feederCommands = Shuffleboard.getTab("Commands")
            .getLayout("Feeder", BuiltInLayouts.kList)
            .withSize(2, 4);

        feederCommands.add(new FeederUpCommand());

    }


    public void climberInfo() {

    }

    public void shooterInfo() {

    }

    public void visionInfo() {

    }



}
