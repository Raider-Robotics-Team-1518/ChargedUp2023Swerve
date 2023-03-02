package frc.robot.commands.operational.setup.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/*
 * Sets max position of telescope (aka fully extended)
 */
public class TelescopeSetMax extends CommandBase {
    public TelescopeSetMax() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        Constants.updateDouble(Constants.TELESCOPE_MAX_POS, RobotContainer.armSubsystem.getTelescopePosition());
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
