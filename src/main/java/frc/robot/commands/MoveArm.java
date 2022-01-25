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

}

