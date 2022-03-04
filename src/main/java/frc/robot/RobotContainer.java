package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.auto.BlueAuto2;
import frc.robot.auto.FirstAuto;
import frc.robot.auto.ThirdAuto;
import frc.robot.auto.ZeroAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.trajectories.Trajectories;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.commands.RunNeck;
import frc.robot.commands.Shoot;
import frc.robot.commands.CommandGroups.ArmIntakeAndFeeder;
import frc.robot.commands.CommandGroups.NeckAndShoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.Constants.Direction;


public class RobotContainer {

  //Subsystems
  public final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  public final Arm arm = new Arm();
  private final Feeder feeder = new Feeder();
  private final Climber climber = new Climber();
  private final Neck neck = new Neck();
  private final Shooter shooter = new Shooter();

  //Joysticks
  private Joystick leftJoystick = new Joystick(Constants.JoystickConstants.leftJoystickPort);
  private Joystick rightJoystick = new Joystick(Constants.JoystickConstants.rightJoystickPort);
  private Joystick secondaryJoystick = new Joystick(Constants.JoystickConstants.secondaryJoystickPort);


  //Buttons
  private final JoystickButton runIntakeButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.runIntakeForwards);
  private final JoystickButton runIntakeBackwardsButton = new JoystickButton(secondaryJoystick,Constants.JoystickConstants.SecondaryJoystick.runIntakeBackwards);
  private final JoystickButton raiseArmButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.raiseArm);
  private final JoystickButton lowerArmButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.lowerArm);
  private final JoystickButton runFeederForwardsButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.runFeederForwards);
  private final JoystickButton runFeederBackwardsButton= new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.runFeederBackwards);
  private final JoystickButton runNeckButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.runNeck);
  private final JoystickButton runNeckBackwardsButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.runNeckBackwards);
  //private final JoystickButton shootButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.runShooter);
  private final JoystickButton raiseClimberButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.raiseClimber);
  private final JoystickButton lowerClimberButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.lowerClimber);
  private final JoystickButton runNeckAndShootButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.runNeckAndShoot);
  private final JoystickButton lowerArmIntakeAndFeederButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.moveArmIntakeAndFeed);

  // Commands
  private final Drive drive = new Drive(drivetrain, leftJoystick, rightJoystick);
  private final MoveArm lowerArm = new MoveArm(arm, ArmState.LOW);
  private final MoveArm raiseArm = new MoveArm(arm, ArmState.HIGH);
  private final RunIntake runIntakeForwards = new RunIntake(intake, Direction.FORWARDS);
  private final RunIntake runIntakeBackwards = new RunIntake(intake, Direction.BACKWARDS);
  private final RunFeeder runFeederForwards = new RunFeeder(feeder, Direction.FORWARDS);
  private final RunFeeder runFeederBackwards = new RunFeeder(feeder, Direction.BACKWARDS);
  private final Climb raiseClimber = new Climb(climber, ClimberState.RAISE);
  private final Climb lowerClimber = new Climb(climber, ClimberState.LOWER);
  private final RunNeck runNeck = new RunNeck(neck, Direction.FORWARDS);
  private final RunNeck runNeckBackwards = new RunNeck(neck, Direction.BACKWARDS);
  private final Shoot shoot = new Shoot(shooter);
  private final NeckAndShoot neckAndShoot = new NeckAndShoot(feeder,neck, shooter);
  private final ArmIntakeAndFeeder armIntakeAndFeeder = new ArmIntakeAndFeeder(arm, intake, feeder);


  public RobotContainer() {
    configureButtonBindings();
    configureAutos();
    zeroSubsystems();
  }

  public void periodic() {

  }  

  public void zeroSubsystems() {
    arm.zeroArm();
    climber.zeroClimber();
  }

  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(drive);
    runIntakeButton.whenHeld(runIntakeForwards);
    //shootButton.whenHeld(shoot);
    runNeckButton.whenHeld(runNeck);
    runNeckBackwardsButton.whenHeld(runNeckBackwards);
    runIntakeBackwardsButton.whenHeld(runIntakeBackwards);
    raiseArmButton.whenHeld(raiseArm);
    lowerArmButton.whenHeld(lowerArm);
    runFeederForwardsButton.whenHeld(runFeederForwards);
    runFeederBackwardsButton.whenHeld(runFeederBackwards);
    raiseClimberButton.whenHeld(raiseClimber);
    lowerClimberButton.whenHeld(lowerClimber);
    runNeckAndShootButton.whenHeld(neckAndShoot);
    lowerArmIntakeAndFeederButton.whenHeld(armIntakeAndFeeder);
    lowerArmIntakeAndFeederButton.whenReleased(new MoveArm(arm, ArmState.HIGH));
    lowerArmButton.whenReleased(new MoveArm(arm, ArmState.HIGH));
  }

  private void configureAutos() {

  }

  public Command getAutonomousCommand() {
    return null;
  }

  public Arm getArm() {
    return arm;
  }

  public Feeder getFeeder() {
    return feeder;
  }

  public Drivetrain getDriveTrain() {
    return drivetrain;
  }

  public Intake getIntake() {
    return intake;
  }

  public Climber getClimber() {
    return climber;
  }

  public Shooter getShooter(){
    return shooter;
  }

  public Neck getNeck() {
    return neck;
  }
}

