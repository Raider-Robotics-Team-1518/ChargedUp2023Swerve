package frc.robot.commands.operational.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Toggles automatic leveling of the wrist
 * When this property is active, the drivers cannot move the wrists by using the controllers, instead, PID automatically fixes the wrist position so that it is always level
 * (We created this concept assuming that always having control over the wrist can be too tedious, when it most of the time just needs to be level) 
 */
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

