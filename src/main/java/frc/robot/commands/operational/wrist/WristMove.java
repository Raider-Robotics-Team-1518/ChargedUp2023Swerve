package frc.robot.commands.operational.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Sets the wrist motor to a static positive speed.
 */
public class WristMove extends CommandBase {
    double speed = 0.375d;
    public WristMove(double speed) {
        this.speed = speed;
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        if(RobotContainer.armSubsystem.isWristInRange() && !RobotContainer.armSubsystem.lockedWrist) {
            RobotContainer.armSubsystem.getWristMotor().set(speed);
        }
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
