package frc.robot.commands.autonomous.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoTelescopeExtend extends CommandBase {
    public AutoTelescopeExtend() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
        //return RobotContainer.armSubsystem.getTelescopePosition();
    }
}
