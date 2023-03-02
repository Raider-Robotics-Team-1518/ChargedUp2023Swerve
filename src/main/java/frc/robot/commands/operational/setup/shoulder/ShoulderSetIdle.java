package frc.robot.commands.operational.setup.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/*
 * Sets the idle position (by some angle measured in degrees) of the shoulder (idle = when robot is stationairy on the field before autonomous)
 */
public class ShoulderSetIdle extends CommandBase {
    public ShoulderSetIdle() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        Constants.updateDouble(Constants.SHOULDER_IDLE_ANGLE, RobotContainer.armSubsystem.getShoulderAngle());
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
