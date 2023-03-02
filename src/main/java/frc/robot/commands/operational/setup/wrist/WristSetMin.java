package frc.robot.commands.operational.setup.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class WristSetMin extends CommandBase {
    private boolean found = false;

    public WristSetMin() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.resetWristPosition();
        Constants.updateDouble(Constants.WRIST_MIN_POS, 0.0d);
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

