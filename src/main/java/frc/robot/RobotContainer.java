package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  //Subsystems
  private Drivetrain drivetrain = new Drivetrain();
  private Intake intake = new Intake();
  private Arm arm = new Arm();

  //Joysticks
  private Joystick leftJoystick = new Joystick(Constants.JoystickConstants.leftJoystickPort);
  private Joystick rightJoystick = new Joystick(Constants.JoystickConstants.rightJoystickPort);
  private Joystick secondaryJoystick = new Joystick(Constants.JoystickConstants.secondaryJoystickPort);

  // TODO:REMOVE ALL EXAMPLE BUTTONS, COMMANDS, METHODS AFTER WEEK 1

  // Buttons
  private final JoystickButton exampleButton = new JoystickButton(secondaryJoystick, 1);
  private final JoystickButton runIntakeButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.runIntakeForward);
  private final JoystickButton runIntakeBackwardsButton = new JoystickButton(secondaryJoystick,Constants.JoystickConstants.SecondaryJoystick.runIntakeBackwards);
  private final JoystickButton raiseArmButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.raiseArm);
  private final JoystickButton lowerArmButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.lowerArm);


  // Commands
  private final Drive drive = new Drive(drivetrain, leftJoystick, rightJoystick);
  private final Drive exampleCommand = new Drive(drivetrain, leftJoystick, rightJoystick);
  private final MoveArm stopArm = new MoveArm(arm, 0); // unused
  private final MoveArm lowerArm = new MoveArm(arm, 1);
  private final MoveArm raiseArm = new MoveArm(arm, 2);
  private final RunIntake runIntake = new RunIntake(intake, true);
  private final RunIntake runIntakeBackwards = new RunIntake(intake, false);

  // Auto
  private final ZeroAuto zeroAuto = new ZeroAuto();
  private final FirstAuto firstAuto = new FirstAuto();
  private final SecondAuto secondAuto = new SecondAuto();
  private final ThirdAuto thirdAuto = new ThirdAuto();

  private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(drive);
    exampleButton.whenHeld(exampleCommand); // Example Code
    runIntakeButton.whenHeld(runIntake);
    runIntakeBackwardsButton.whenHeld(runIntakeBackwards);
    raiseArmButton.whenHeld(raiseArm);
    lowerArmButton.whenHeld(lowerArm);

  }

  private void configureAutos() {
    // TODO: Rename Autos on Dashboard

    sendableChooser.setDefaultOption("No Auto", zeroAuto);
    sendableChooser.addOption("First Auto", firstAuto);
    sendableChooser.addOption("Second Auto", secondAuto);
    sendableChooser.addOption("Third Auto", thirdAuto);
  }

  public Command getAutonomousCommand() {

    return sendableChooser.getSelected();
  }
  
}
