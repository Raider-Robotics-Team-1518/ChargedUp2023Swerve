package frc.robot.commands.autonomous.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoRotateWristGrabbing extends CommandBase {
    public AutoRotateWristGrabbing() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.lockedWrist = true;
        RobotContainer.armSubsystem.setWristTargetPos(90+RobotContainer.armSubsystem.getShoulderAngle(), true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.wristPidController.atSetpoint();
    }
}
