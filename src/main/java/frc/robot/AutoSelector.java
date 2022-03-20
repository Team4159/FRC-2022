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
import frc.robot.auto.Blue1Ball;
import frc.robot.auto.Red2Ball;
import frc.robot.auto.Red3Ball;
import frc.robot.auto.Red1Ball;
import frc.robot.auto.Blue2Ball;
import frc.robot.auto.Blue3Ball;


public class AutoSelector {
    private ShuffleboardTab preMatch = Shuffleboard.getTab("Pre-Match");

   // Auto
    private final SendableChooser<Command> autoSelector = new SendableChooser<>();
    private final Red1Ball redAuto6;
    private final Blue2Ball blueAuto10;
    private final Blue1Ball blueAuto6;
    private final Red2Ball redAuto10;
   

    public AutoSelector(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        redAuto6 = new Red1Ball(drivetrain, arm, intake, feeder, neck, shooter);
        redAuto10 = new Red2Ball(drivetrain, arm, intake, feeder, neck, shooter);
        blueAuto6 = new Blue1Ball(drivetrain, arm, intake, feeder, neck, shooter);
        blueAuto10 = new Blue2Ball(drivetrain, arm, intake, feeder, neck, shooter);
        configureAutoSelector();
        
    }

    public void configureAutoSelector() {
        autoSelector.addOption("Red-Auto-1-Ball", redAuto6);
        autoSelector.addOption("Red-Auto-2-Ball", redAuto10);
        autoSelector.addOption("Blue-Auto-1-Ball", blueAuto6);
        autoSelector.addOption("Blue-Auto-2-Ball", blueAuto10);
        Shuffleboard.getTab("Pre-Match")
            .add("Auto Selector", autoSelector)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 2)
            .withPosition(0, 0);
    }

    public Command getSelectedAuto() {
        return autoSelector.getSelected().withTimeout(15);
    }


}
