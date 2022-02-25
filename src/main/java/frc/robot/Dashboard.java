package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.FirstAuto;
import frc.robot.auto.SecondAuto;
import frc.robot.auto.ThirdAuto;
import frc.robot.auto.ZeroAuto;

public class Dashboard {
    private ShuffleboardTab driveTeam = Shuffleboard.getTab("Drive Team");
    private ShuffleboardTab electrical = Shuffleboard.getTab("Electrical");
    private ShuffleboardTab testing = Shuffleboard.getTab("Testing");
    Shuffleboard.selectTab("Drive Team");


   // Auto
   private final ZeroAuto zeroAuto = new ZeroAuto();
   private final FirstAuto firstAuto = new FirstAuto();
   private final SecondAuto secondAuto = new SecondAuto();
   private final ThirdAuto thirdAuto = new ThirdAuto();
   private CommandBase selectedAuto;
   private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

   // Add commands to the autonomous command chooser
   

    public Dashboard() {
        configureAutos();
        display();
    }

    public void configureAutos() {
        sendableChooser.setDefaultOption("No Auto", zeroAuto);
        sendableChooser.addOption("First Auto", firstAuto);
        sendableChooser.addOption("Second Auto", secondAuto);
        sendableChooser.addOption("Third Auto", thirdAuto);
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

    public CommandBase getSelectedAuto() {
        return selectedAuto;
    }


}


    public void visionInfo() {

    }



}
