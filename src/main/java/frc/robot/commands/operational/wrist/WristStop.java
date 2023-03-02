package frc.robot.commands.operational.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Stops movement of the wrist motor.
 */
public class WristStop extends CommandBase {
    public WristStop() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.getWristMotor().set(0.0d);
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
        return false;
    }
}
