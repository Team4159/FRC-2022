package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Direction;
import frc.robot.commands.AutoCommands.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmState;


/*
    Runs a ~ 30 second test before we switch the battery after every match to ensure all subsystems + commands are running correctly
    Test once you enable the test mode in Driverstation

    Commands not tested: Drive (needs joystick input)
*/

public class Test extends SequentialCommandGroup{
    //Subsystems
    private Arm arm;
    private Climber climber;
    private Drivetrain drivetrain;
    private Feeder feeder;
    private Intake intake;
    private Neck neck;
    private Shooter shooter;
    //AutoCommands
    private MoveDistance moveDistance;
    private TurnDegrees turnDegrees;
    //CommandGroups
    private Climb climbRaise;
    private Climb climbLower;
    private MoveArm moveArmLow;
    private MoveArm moveArmHigh;
    private RunFeeder runFeederForwards;
    private RunFeeder runFeederBackwards;
    //Commands
    //Variables
    private Timer timer = new Timer();
    private double outputValue;
    private String fail = "";
    
    public Test(Arm arm, Climber climber, Drivetrain drivetrain, Feeder feeder, Intake intake, Neck neck, Shooter shooter) {
        //subsystems
        this.arm = arm;
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.intake = intake;
        this.neck = neck;
        this.shooter = shooter;
        //AutoCommands
        this.moveDistance = MoveDistance(drivetrain, 3); //TODO: figure out how far that 3 is
        this.turnDegrees = TurnDegrees(drivetrain, 360);
        //CommandGroups
        this.climbRaise = Climb(climber, Climber.ClimberState.RAISE);
        this.climbLower = Climb(climber, Climber.ClimberState.LOWER);
        this.moveArmHigh = MoveArm(arm, Arm.ArmState.HIGH);
        this.moveArmLow = MoveArm(arm, Arm.ArmState.LOW);
        this.runFeederForwards = RunFeeder(feeder, Constants.Direction.FORWARDS);
        this.runFeederBackwards = RunFeeder(feeder, Constants.Direction.BACKWARDS);
        timer.start();
    }

    public void Test() {
        //Ordered based on the order in vscode (alphabetical inside the robot folder)
        //AutoCommands
        if(timer.get() < 1) {
            moveDistance.execute();
        }
        else if(timer.get() == 1) {
            if(!moveDistance.isFinished()) { //TODO: isFinished might run early and could give false results but without the robot I can't test it
                System.out.println("------------------------------");
                System.out.println("MoveDistance: BROKEN \u2715");
                fail += "MoveDistance, ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("MoveDistance: OK \u2713");
            }
        }
        
        else if(timer.get() < 2) {
            turnDegrees.execute();
        }
        else if(timer.get() == 2) {
            if(!turnDegrees.isFinished()) {
                System.out.println("------------------------------");
                System.out.println("TurnDegrees: BROKEN \u2715");
                fail += "TurnDegrees, ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("TurnDegrees: OK \u2713");
            }
        }

        //CommandGroups
        else if(timer.get() < 3) {
            climbRaise.execute();
        }
        else if(timer.get() == 3) {
            if(!tolerance(climber.getEncoderRaw(), climber.getHighSetPoint(), 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Climb (Raise): BROKEN \u2715");
                fail += "Climb (Raise), ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("Climb (Raise): OK \u2713");
            }
        }

        else if(timer.get() < 4) {
            climbLower.execute();
        }
        else if(timer.get() == 4) {
            if(!tolerance(climber.getEncoderRaw(), climber.getLowSetPoint(), 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Climb (Lower): BROKEN \u2715");
                fail += "Climb (Lower), ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("Climb (Lower): OK \u2713");
            }
        }
        
        else if(timer.get() < 5) {
            moveArmHigh.execute();
        }
        else if(timer.get() == 5) {
            if(!arm.atSetpoint(Constants.IntakeAndArmConstants.pidHighSetPoint, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Arm (High): BROKEN \u2715");
                fail += "Arm (High), ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("Arm (High): OK \u2713");
            }
        }

        else if(timer.get() < 6) {
            moveArmLow.execute();
        }
        else if(timer.get() == 6) {
            if(!arm.atSetpoint(Constants.IntakeAndArmConstants.pidLowSetPoint, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Arm (Low): BROKEN \u2715");
                fail += "Arm (Low), ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("Arm (Low): OK \u2713");
            }
        }

        //TODO: Feeder doesn't actually have anything to check
        //Please somebody find something to check
        else if(timer.get() < 7) {
            runFeederForwards.execute();
        }
        else if(timer.get() < 8) {
            runFeederBackwards.execute();
        }












        
        //below stuff is kinda broken probably maybe not?
        //I won't touch it for now but I'll edit it once I get back to subsystems
        
        if(timer.get() < 1) {
            intake.runIntake(Direction.FORWARDS);
            //spin up motor
            // System.out.println(tolerance(intake.getIntakeSpark().get(), Constants.IntakeAndArmConstants.intakeSpeed, 0.01));
            //^ will get annoying after a while, might be good for debugging though
        }
        else if(timer.get() == 1) {
            //give the motor time to spin up and then check if the final speed is within expected levels
            if(!tolerance(intake.getIntakeSpark().get(), Constants.IntakeAndArmConstants.intakeSpeed, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Intake Forwards: BROKEN \u2715");
                System.out.println("------------------------------");
                fail += "Intake Forwards, ";
            }
            else {
                System.out.print("------------------------------");
                System.out.print("Intake Forwards: OK \u2713");
                System.out.print("------------------------------");
            }
            //stop intake to prepare for next test
            intake.stop();
        }

        else if(timer.get() < 2) {
            intake.runIntake(Direction.BACKWARDS);
        }
        else if(timer.get() == 2) {
            if(!tolerance(intake.getIntakeSpark().get(), -Constants.IntakeAndArmConstants.intakeSpeed, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Intake Backwards: BROKEN \u2715");
                System.out.println("------------------------------");
                fail += "Intake Backwards, ";
            }
            else {
                System.out.println("------------------------------");
                System.out.println("Intake Backwards: OK \u2713");
                System.out.println("------------------------------");
            }
            //stop intake to prepare for next test
            intake.stop();
        }

        else if(timer.get() < 3) {
            arm.runArm(ArmState.HIGH);
        }
        else if(timer.get() == 3) {
            if(!tolerance(arm.getArmSpark().get(), Constants.IntakeAndArmConstants.pidHighSetPoint, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Arm High/Up: BROKEN \u2715");
                System.out.println("------------------------------");
                fail += "Arm High/Up, ";
            }
            else {
                System.out.println("------------------------------");
                System.out.println("Arm High/Up: OK \u2713");
                System.out.println("------------------------------");
            }
        }
        
        else if(timer.get() < 4) {
            arm.runArm(ArmState.LOW);
        }
        else if (timer.get() == 4) {
            if(!tolerance(arm.getArmSpark().get(), Constants.IntakeAndArmConstants.pidLowSetPoint, 0.01)) {
                System.out.println("------------------------------");
                System.out.println("Arm Low/Down: BROKEN \u2715");
                System.out.println("------------------------------");
                fail += "Arm Low/Down, ";
            }
            else {
                System.out.println("------------------------------");
                System.out.println("Arm Low/Down: OK \u2713");
                System.out.println("------------------------------");
            }
        }

        else if(timer.get() < 5) {

        }

        //display all broken subsystems
        else if (timer.get() == 30) {//TODO: 30 is temporary
            System.out.println("Broken Commands/Subsystems: " + fail);
        }
    }

    public boolean tolerance(double value, double expectedValue, double tolerance) {
        return value <= expectedValue + tolerance && value >= expectedValue - tolerance;
    }
}
