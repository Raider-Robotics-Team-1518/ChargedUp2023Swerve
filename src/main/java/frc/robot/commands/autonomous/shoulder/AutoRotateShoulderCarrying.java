package frc.robot.commands.autonomous.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoRotateShoulderCarrying extends CommandBase {
    public AutoRotateShoulderCarrying() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setShoulderTargetPos(Constants.ARM_SHOULDER_LOWERSWITCH_DEG+45, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.shoulderPidController.atSetpoint();
    }
}
