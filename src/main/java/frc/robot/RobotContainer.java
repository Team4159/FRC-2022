package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.trajectories.Trajectories;
import frc.robot.commands.AutoCommands.MoveDistance;
import frc.robot.commands.AutoCommands.TurnDegrees;
import frc.robot.commands.CommandGroups.ArmIntakeAndFeeder;
import frc.robot.commands.CommandGroups.MoveArmAndIntakeBackwards;
import frc.robot.commands.CommandGroups.NeckAndFeeder;
import frc.robot.commands.CommandGroups.NeckAndShoot;
import frc.robot.commands.CommandGroups.*;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.Constants.Direction;
import frc.robot.Constants.JoystickConstants;
import frc.robot.auto.ZeroAuto;

public class RobotContainer {
  final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
  private final SingleMotorSubsystem intake = new SingleMotorSubsystem(Constants.CanIds.intakeSpark, Constants.IntakeAndArmConstants.intakeSpeed);
  private final SingleMotorSubsystem feeder = new SingleMotorSubsystem(Constants.CanIds.feederSpark, Constants.FeederConstants.feederSpeed);
  private final SingleMotorSubsystem neck = new SingleMotorSubsystem(Constants.CanIds.neckSpark, Constants.NeckConstants.neckSpeed);
  private final Shooter shooter = new Shooter();
  // private final Climber climber = new Climber();

  //Joysticks
  public Joystick leftJoystick = new Joystick(Constants.JoystickConstants.leftJoystickPort);
  public Joystick rightJoystick = new Joystick(Constants.JoystickConstants.rightJoystickPort);
  public Joystick secondaryJoystick = new Joystick(Constants.JoystickConstants.secondaryJoystickPort);

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
  private final JoystickButton shootInButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.shootIn);
  private final JoystickButton lowerArmIntakeAndFeederButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.moveArmIntakeAndFeed);
  private final JoystickButton feederAndNeckButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.feederAndNeck);
  private final JoystickButton halfPowerButton = new JoystickButton(rightJoystick, Constants.JoystickConstants.RightJoystick.halfPowerToggle);
  private final JoystickButton flipDrivetrainToggleButton = new JoystickButton(rightJoystick, Constants.JoystickConstants.RightJoystick.flipDrivetrainToggle);
  private final JoystickButton moveArmAndIntakeBackwardsButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.moveArmAndIntakeBackwards);
  private final JoystickButton climbAutoButton = new JoystickButton(rightJoystick, Constants.JoystickConstants.RightJoystick.climbAuto);
  private final JoystickButton vomitButton = new JoystickButton(secondaryJoystick, Constants.JoystickConstants.SecondaryJoystick.vomit);

  // Commands
  private final Drive drive = new Drive(drivetrain, leftJoystick, rightJoystick, Drivetrain.PowerOutput.FULL_POWER);
  private final MoveArm lowerArm = new MoveArm(arm, ArmState.LOW);
  private final MoveArm raiseArm = new MoveArm(arm, ArmState.HIGH);
  private final RunIntake runIntakeForwards = new RunIntake(intake, Direction.FORWARDS);
  private final RunIntake runIntakeBackwards = new RunIntake(intake, Direction.BACKWARDS);
  private final RunFeeder runFeederForwards = new RunFeeder(feeder, Direction.FORWARDS);
  private final RunFeeder runFeederBackwards = new RunFeeder(feeder, Direction.BACKWARDS);
  // private final Climb raiseClimber = new Climb(climber, ClimberState.RAISE);
  // private final Climb lowerClimber = new Climb(climber, ClimberState.LOWER);
  // private final ClimbArm climbArm = new ClimbArm(climber, secondaryJoystick);
  private final Command neckFwds = new InstantCommand(() -> neck.run(Direction.FORWARDS), neck);
  private final RunNeck runNeckBackwards = new RunNeck(neck, Direction.BACKWARDS);
  private final Shoot shoot = new Shoot(shooter, Constants.ShooterConstants.targetVelocity);
  private final ShootIn shootIn = new ShootIn(shooter);
  private final MoveNeckAndFeederBackwards vomit = new MoveNeckAndFeederBackwards(feeder, neck);
  private final NeckAndShoot neckAndShoot = new NeckAndShoot(feeder,neck, shooter);
  private final ArmIntakeAndFeeder armIntakeAndFeeder = new ArmIntakeAndFeeder(arm, intake, feeder,neck);
  private final HalfPower halfPower = new HalfPower(drivetrain, true);
  private final HalfPower fullPower = new HalfPower(drivetrain, false);
  private final Command flipDrivetrain = new InstantCommand(() -> drivetrain.flipDrivetrain(), drivetrain);
  private final NeckAndFeeder feederAndNeck = new NeckAndFeeder(feeder, neck);
  private final MoveArmAndIntakeBackwards moveArmAndIntakeBackwards = new MoveArmAndIntakeBackwards(arm, intake, feeder, neck);
  private final AutoClimb autoClimb = new AutoClimb(drivetrain, true);
  private final AutoClimb stopAutoClimb = new AutoClimb(drivetrain, false);
  private AutoSelector autoSelector = new AutoSelector(drivetrain, arm, intake, feeder, neck, shooter);

  public RobotContainer() {
    configureButtonBindings();
    zeroSubsystems();

    arm.setDefaultCommand(new MoveArm(arm, ArmState.HIGH));
  }

  public void zeroSubsystems() {
    arm.zeroArm();
    drivetrain.zeroSensors();
    // climber.zeroClimber();
    drivetrain.zeroSensors();
  }

  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(drive);
    // climber.setDefaultCommand(climbArm);
    arm.setDefaultCommand(new MoveArm(arm, ArmState.HIGH));
    runIntakeButton.whileTrue(runIntakeForwards);
    //shootButton.whileTrue(shoot);
    runNeckButton.whileTrue(neckFwds);
    runNeckBackwardsButton.whileTrue(runNeckBackwards);
    runIntakeBackwardsButton.whileTrue(runIntakeBackwards);
    raiseArmButton.whileTrue(raiseArm);
    lowerArmButton.whileTrue(lowerArm);
    runFeederForwardsButton.whileTrue(runFeederForwards);
    runFeederBackwardsButton.whileTrue(runFeederBackwards);
    // raiseClimberButton.onTrue(raiseClimber);
    // lowerClimberButton.onTrue(lowerClimber);
    runNeckAndShootButton.onTrue(neckAndShoot);
    runNeckAndShootButton.whileTrue(shoot);
    shootInButton.whileTrue(shootIn); 
    vomitButton.whileTrue(vomit);
    lowerArmIntakeAndFeederButton.whileTrue(armIntakeAndFeeder);
    lowerArmIntakeAndFeederButton.onFalse(new MoveArm(arm, ArmState.HIGH));
    lowerArmIntakeAndFeederButton.onFalse(new RunNeck(neck, Direction.BACKWARDS).withTimeout(0.5));
    lowerArmIntakeAndFeederButton.onFalse(new RunFeeder(feeder, Direction.FORWARDS).withTimeout(0.1)); //0.05
    halfPowerButton.onTrue(halfPower);
    halfPowerButton.onFalse(fullPower);
    flipDrivetrainToggleButton.onTrue(flipDrivetrain);
    feederAndNeckButton.whileTrue(feederAndNeck);
    moveArmAndIntakeBackwardsButton.whileTrue(moveArmAndIntakeBackwards);
    climbAutoButton.onTrue(autoClimb);
    climbAutoButton.onFalse(stopAutoClimb);
    //lowerArmButton.onFalse(new MoveArm(arm, ArmState.HIGH));
  }
  
  public Command getAutonomousCommand() {
    return autoSelector.getSelectedAuto();
  }
}
