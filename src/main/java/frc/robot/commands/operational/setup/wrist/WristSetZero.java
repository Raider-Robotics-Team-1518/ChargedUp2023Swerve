package frc.robot.commands.operational.setup.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Sets the zero position of the wrist, idealy this is when the wrist is facing fully horizontal, and level (90 degrees)
 */
public class WristSetZero extends CommandBase {
    public WristSetZero() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.resetWristPosition();
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

