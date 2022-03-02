package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

public class LowerArm extends CommandBase{
    
    private Arm arm;

    public LowerArm(Arm arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.runArm(ArmState.LOW);
    }
}
