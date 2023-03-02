package frc.robot.commands.operational.setup.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Sets the zero position of the shoulder, idealy this is when the shoulder is facing fully horizontal, and level (90 degrees)
 */
public class ShoulderSetZero extends CommandBase {
    public ShoulderSetZero() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.resetShoulderPosition();
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
