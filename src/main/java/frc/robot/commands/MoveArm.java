package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeAndArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;


public class MoveArm extends CommandBase{
    
    private Arm arm;

    private ArmState armState;

    public MoveArm(Arm arm, ArmState armState) {
        this.arm = arm;
        this.armState = armState;
        addRequirements(arm);

    }

    @Override
    public void execute() {
        arm.runArm(armState);
    }

    // public boolean isFinished() {
    //     double setpoint = 0;
    //     if(armState == ArmState.LOW) {
    //         setpoint = Constants.IntakeAndArmConstants.pidLowSetPoint;
    //     }
    //     return arm.atSetpoint(setpoint, Constants.IntakeAndArmConstants.tolerance);
    // }

    @Override
    public void end(boolean i) {
        // System.out.println(arm.atSetpoint(Constants.IntakeAndArmConstants.pidHighSetPoint, Constants.IntakeAndArmConstants.tolerance));
        // if(armState == ArmState.HIGH && arm.atSetpoint(Constants.IntakeAndArmConstants.pidHighSetPoint, Constants.IntakeAndArmConstants.tolerance)) {
        //     arm.setArmSpeed(0);
        //     //System.out.println("true");
        // }
        // else {
        //     arm.runArm(armState.HIGH);
        //     //System.out.println("false");
        // }
    }

}
