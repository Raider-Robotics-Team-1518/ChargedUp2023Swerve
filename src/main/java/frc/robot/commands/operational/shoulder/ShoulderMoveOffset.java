package frc.robot.commands.operational.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShoulderMoveOffset extends CommandBase {
    double opp = 0;
    public ShoulderMoveOffset(double offset) {
        this.opp = offset;
        addRequirements(RobotContainer.armSubsystem);
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.shoulderPidController.setSetpoint(RobotContainer.armSubsystem.shoulderPidController.getSetpoint()+opp);
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
        return RobotContainer.armSubsystem.shoulderPidController.atSetpoint();
    }
  }

