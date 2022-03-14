package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.auto.Blue6;
import frc.robot.auto.Red10;
import frc.robot.auto.Red6;
import frc.robot.auto.Blue10;


public class AutoSelector {
    private ShuffleboardTab preMatch = Shuffleboard.getTab("Pre-Match");

   // Auto
    private final SendableChooser<Command> autoSelector = new SendableChooser<>();
    private final Red6 redAuto6;
    private final Blue10 blueAuto10;
    private final Blue6 blueAuto6;
    private final Red10 redAuto10;
   

    public AutoSelector(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        redAuto6 = new Red6(drivetrain, arm, intake, feeder, neck, shooter);
        redAuto10 = new Red10(drivetrain, arm, intake, feeder, neck, shooter);
        blueAuto6 = new Blue6(drivetrain, arm, intake, feeder, neck, shooter);
        blueAuto10 = new Blue10(drivetrain, arm, intake, feeder, neck, shooter);
        configureAutoSelector();
        
    }

    public void configureAutoSelector() {
        autoSelector.addOption("Red-Auto-6", redAuto6);
        autoSelector.addOption("Red-Auto-10", redAuto10);
        autoSelector.addOption("Blue-Auto-6", blueAuto6);
        autoSelector.addOption("Blue-Auto-10", blueAuto10);
        Shuffleboard.getTab("Pre-Match")
            .add("Auto Selector", autoSelector)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 2)
            .withPosition(0, 0);
    }

    public Command getSelectedAuto() {
        return autoSelector.getSelected();
    }


}
