package frc.robot.commands.operational.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShoulderMoveOffset extends CommandBase {
    double opp = 0;
    public ShoulderMoveOffset(double offset) {
        this.opp = offset;
        addRequirements(RobotContainer.armSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.armSubsystem.shoulderPidController.setSetpoint(RobotContainer.armSubsystem.shoulderPidController.getSetpoint()+opp);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.shoulderPidController.atSetpoint();
    }
  }

