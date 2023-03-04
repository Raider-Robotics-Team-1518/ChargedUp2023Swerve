package frc.robot.commands.autonomous.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Rotate arm to prime dropping position
 */

public class AutoRotateShoulderPlacing extends CommandBase {
    public AutoRotateShoulderPlacing() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setShoulderTargetPos(90, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.shoulderPidController.atSetpoint();
    }
}
