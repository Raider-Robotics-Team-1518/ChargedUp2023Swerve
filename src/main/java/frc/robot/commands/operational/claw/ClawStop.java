package frc.robot.commands.operational.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Stops all movement of the claw
 */
public class ClawStop extends CommandBase {
    public ClawStop() {
        addRequirements(RobotContainer.clawSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.clawSubsystem.setClawSpeed(0.0d);
    }
  
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
  
    @Override
    public boolean isFinished() {
        return true;
    }
}
