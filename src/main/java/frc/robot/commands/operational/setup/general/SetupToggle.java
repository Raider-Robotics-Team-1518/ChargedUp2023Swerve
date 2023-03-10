package frc.robot.commands.operational.setup.general;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/*
 * Enables Team 1518's Setup mode
 * What: Toggles various debug information that outputs to SmartDashboard
 * When: Preferably after everytime the robot has encountered changes (very small or very big, does not matter) in positioning of mechanical parts
 * Why: Without proper setup, the arm, which is, keep in mind, a moving object, 
 */
public class SetupToggle extends CommandBase {
    public SetupToggle() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        Constants.setupState = !Constants.setupState;
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

