package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoFeedClaw extends CommandBase {
    public AutoFeedClaw() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.clawSubsystem.setClawSpeed(Constants.clawFeedSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
