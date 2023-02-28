package frc.robot.commands.operational.setup.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WristToggleLock extends CommandBase {
    public WristToggleLock() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.lockedWrist = !RobotContainer.armSubsystem.lockedWrist;
    }
  
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }
}

