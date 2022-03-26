package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Drivetrain.PowerOutput;
import frc.robot.trajectories.Trajectories;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.commands.AutoCommands.MoveDistance;
import frc.robot.commands.AutoCommands.TurnDegrees;
import frc.robot.commands.CommandGroups.ArmIntakeAndFeeder;
import frc.robot.commands.CommandGroups.NeckAndShoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.Constants.Direction;
import frc.robot.auto.ZeroAuto;

public class RobotContainer {
  //Power Distribution
  private final PowerDistribution PDH = new PowerDistribution();
  //Subsystems, will be accessed with getters here in the same file
  final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  public final Arm arm = new Arm();
  private final Feeder feeder = new Feeder();
  // private final Climber climber = new Climber();
  private final Neck neck = new Neck();
  private final Shooter shooter = new Shooter();

  //Joysticks
  public Joystick leftJoystick = new Joystick(Constants.JoystickConstants.leftJoystickPort);
  public Joystick rightJoystick = new Joystick(Constants.JoystickConstants.rightJoystickPort);
  public Joystick secondaryJoystick = new Joystick(Constants.JoystickConstants.secondaryJoystickPort);

  //Joystick getters
  public Joystick getLJoystick() {
    return leftJoystick;
  }
  public Joystick getRJoystick(){
    return rightJoystick;
  }
  public Joystick getSecJoystick(){
    return secondaryJoystick;
  }

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

  private final JoystickButton halfPowerButton = new JoystickButton(rightJoystick, Constants.JoystickConstants.RightJoystick.halfPowerToggle);
  private final JoystickButton flipDrivetrainToggleButton = new JoystickButton(rightJoystick, Constants.JoystickConstants.RightJoystick.flipDrivetrainToggle);

  // Commands
  private final Drive drive = new Drive(drivetrain, leftJoystick, rightJoystick, PowerOutput.FULL_POWER);
  private final MoveArm lowerArm = new MoveArm(arm, ArmState.LOW);
  private final MoveArm raiseArm = new MoveArm(arm, ArmState.HIGH);
  private final RunIntake runIntakeForwards = new RunIntake(intake, Direction.FORWARDS);
  private final RunIntake runIntakeBackwards = new RunIntake(intake, Direction.BACKWARDS);
  private final RunFeeder runFeederForwards = new RunFeeder(feeder, Direction.FORWARDS);
  private final RunFeeder runFeederBackwards = new RunFeeder(feeder, Direction.BACKWARDS);
  // private final Climb raiseClimber = new Climb(climber, ClimberState.RAISE);
  // private final Climb lowerClimber = new Climb(climber, ClimberState.LOWER);
  private final RunNeck runNeck = new RunNeck(neck, Direction.FORWARDS);
  private final RunNeck runNeckBackwards = new RunNeck(neck, Direction.BACKWARDS);
  private final Shoot shoot = new Shoot(shooter, Constants.ShooterConstants.targetVelocity);
  private final NeckAndShoot neckAndShoot = new NeckAndShoot(feeder,neck, shooter);
  private final ArmIntakeAndFeeder armIntakeAndFeeder = new ArmIntakeAndFeeder(arm, intake, feeder,neck);
  private final HalfPower halfPower = new HalfPower(drivetrain, true);
  private final HalfPower fullPower = new HalfPower(drivetrain, false);
  private final FlipDrivetrain flipDrivetrain = new FlipDrivetrain(drivetrain);

  private AutoSelector autoSelector = new AutoSelector(drivetrain, arm, intake, feeder, neck, shooter);



  public RobotContainer() {
    configureButtonBindings();
    zeroSubsystems();
  }

  public void periodic() {

  }  

  public void zeroSubsystems() {
    arm.zeroArm();
    drivetrain.zeroSensors();
    // climber.zeroClimber();
    drivetrain.zeroSensors();
  }

  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(drive);
    arm.setDefaultCommand(new MoveArm(arm, ArmState.HIGH));
    runIntakeButton.whenHeld(runIntakeForwards);
    //shootButton.whenHeld(shoot);
    runNeckButton.whenHeld(runNeck);
    runNeckBackwardsButton.whenHeld(runNeckBackwards);
    runIntakeBackwardsButton.whenHeld(runIntakeBackwards);
    raiseArmButton.whenHeld(raiseArm);
    lowerArmButton.whenHeld(lowerArm);
    runFeederForwardsButton.whenHeld(runFeederForwards);
    runFeederBackwardsButton.whenHeld(runFeederBackwards);
    // raiseClimberButton.whenHeld(raiseClimber);
    // lowerClimberButton.whenHeld(lowerClimber);
    runNeckAndShootButton.whenPressed(neckAndShoot);
    runNeckAndShootButton.whenHeld(shoot);
    lowerArmIntakeAndFeederButton.whenHeld(armIntakeAndFeeder);
    lowerArmIntakeAndFeederButton.whenReleased(new MoveArm(arm, ArmState.HIGH));
    lowerArmIntakeAndFeederButton.whenReleased(new RunNeck(neck, Direction.BACKWARDS).withTimeout(0.5));
    lowerArmIntakeAndFeederButton.whenReleased(new RunFeeder(feeder, Direction.BACKWARDS).withTimeout(0.05));
    halfPowerButton.whenPressed(halfPower);
    halfPowerButton.whenReleased(fullPower);
    flipDrivetrainToggleButton.whenPressed(flipDrivetrain);

    //lowerArmButton.whenReleased(new MoveArm(arm, ArmState.HIGH));

  }
  
  public Command getAutonomousCommand() {

    return autoSelector.getSelectedAuto();

  }

  //Getters for the subsystems
  public Arm getArm() {
    return arm;
  }
  public Drivetrain getDriveTrain() {
    return drivetrain;
  }

  public Intake getIntake() {
    return intake;
  }

  public Feeder getFeeder() {
    return feeder;
  }

  // public Climber getClimber() {
    // return climber;
  // }


  public Shooter getShooter(){
    return shooter;
  }

  public Neck getNeck(){
    return neck;
  }
  //Getter for power distribtion
  public PowerDistribution getPDP(){
    return PDH;
  }
}
