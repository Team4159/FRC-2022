package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase{
    
    private Arm arm;

    public static enum ArmState {
        STOP,
        LOW,
        HIGH
    }
    
    private ArmState armState = ArmState.STOP;

    private int lowSetPoint = Constants.IntakeAndArmConstants.pidLowSetPoint, highSetPoint = Constants.IntakeAndArmConstants.pidHighSetPoint;

    public MoveArm(Arm arm, ArmState armState) {
        this.arm = arm;
        this.armState = armState;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        switch (armState) {
            case STOP:
                arm.stopArm();
                break;
            case LOW:
                arm.runArm(lowSetPoint);
                break;
            case HIGH:
                arm.runArm(highSetPoint);
                break;
        }

    }

}

