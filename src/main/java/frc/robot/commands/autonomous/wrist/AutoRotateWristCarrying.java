package frc.robot.commands.autonomous.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoRotateWristCarrying extends CommandBase {
    public AutoRotateWristCarrying() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.lockedWrist = false;
        RobotContainer.armSubsystem.setWristTargetPos(135, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.wristPidController.atSetpoint();
    }
}
