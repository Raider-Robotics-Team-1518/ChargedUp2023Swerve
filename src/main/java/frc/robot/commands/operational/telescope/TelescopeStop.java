package frc.robot.commands.operational.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Stops movement of the telescope motor.
 */
public class TelescopeStop extends CommandBase {
    public TelescopeStop() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.setTelescopeSpeed(0.0d);
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
