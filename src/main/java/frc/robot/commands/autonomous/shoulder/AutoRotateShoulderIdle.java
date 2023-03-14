package frc.robot.commands.autonomous.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoRotateShoulderIdle extends CommandBase {
    public AutoRotateShoulderIdle() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setShoulderTargetPos(0, false);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

