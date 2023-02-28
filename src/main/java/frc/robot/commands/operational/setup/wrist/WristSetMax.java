package frc.robot.commands.operational.setup.wrist;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class WristSetMax extends CommandBase {
    private boolean found = false;

    public WristSetMax() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        Preferences.setDouble(Constants.WRIST_MAX_POS, RobotContainer.armSubsystem.getWristPosition());
        found = true;
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
        return found;
    }
}

