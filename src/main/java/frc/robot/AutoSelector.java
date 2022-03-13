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
import frc.robot.auto.BlueAuto1;
import frc.robot.auto.BlueAuto2;
import frc.robot.auto.RedAuto3;
import frc.robot.auto.RedAuto1;
import frc.robot.auto.RedAuto2;
import frc.robot.auto.BlueAuto3;


public class AutoSelector {
    private ShuffleboardTab preMatch = Shuffleboard.getTab("Pre-Match");

   // Auto
    private final SendableChooser<Command> autoSelector = new SendableChooser<>();
    private final RedAuto1 redAuto1;
    private final RedAuto2 redAuto2;
    private final BlueAuto3 blueAuto3;
    private final BlueAuto1 blueAuto1;
    private final BlueAuto2 blueAuto2;
    private final RedAuto3 redAuto3;
   

    public AutoSelector(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        redAuto1 = new RedAuto1(drivetrain, arm, intake, feeder, neck, shooter);
        redAuto2 = new RedAuto2(drivetrain, arm, intake, feeder, neck, shooter);
        redAuto3 = new RedAuto3(drivetrain, arm, intake, feeder, neck, shooter);
        blueAuto1 = new BlueAuto1(drivetrain, arm, intake, feeder, neck, shooter);
        blueAuto2 = new BlueAuto2(drivetrain, arm, intake, feeder, neck, shooter);
        blueAuto3 = new BlueAuto3(drivetrain, arm, intake, feeder, neck, shooter);
        configureAutoSelector();
        
    }

    public void configureAutoSelector() {
        autoSelector.addOption("Red-Auto-1", redAuto1);
        autoSelector.addOption("Red-Auto-2", redAuto2);
        autoSelector.addOption("Red-Auto-3", redAuto3);
        autoSelector.addOption("Blue-Auto-1", blueAuto1);
        autoSelector.addOption("Blue-Auto-2", blueAuto2);
        autoSelector.addOption("Blue-Auto-3", blueAuto3);
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
