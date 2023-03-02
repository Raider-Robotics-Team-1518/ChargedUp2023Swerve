package frc.robot.commands.operational.setup.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/*
 * Sets minimum of the telescope (aka fully retracted)
 */
public class TelescopeSetMin extends CommandBase {
    public TelescopeSetMin() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.resetTelescopePosition();
        Constants.updateDouble(Constants.TELESCOPE_MIN_POS, 0.0d);
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
