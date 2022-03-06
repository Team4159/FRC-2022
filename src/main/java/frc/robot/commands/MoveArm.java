package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

    public boolean isFinished() {
        double setpoint;
        if(armState == armState.HIGH) {
            setpoint = Constants.IntakeAndArmConstants.pidHighSetPoint;
        }
        else {
            setpoint = Constants.IntakeAndArmConstants.pidLowSetPoint;
        }
        if(arm.atSetpoint(setpoint, Constants.IntakeAndArmConstants.tolerance)) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean i) {
        if(armState == ArmState.HIGH && arm.atSetpoint(Constants.IntakeAndArmConstants.pidHighSetPoint, Constants.IntakeAndArmConstants.tolerance)) {
            arm.setArmSpeed(0);
            //System.out.println("true");
        }
        else {
            arm.runArm(ArmState.HIGH);
            //System.out.println("false");
        }
        //arm.setArmSpeed(0);
    }

}