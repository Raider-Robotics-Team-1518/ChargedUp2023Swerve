package frc.robot.commands.autonomous.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoRotateShoulderToAngle extends CommandBase {
    private final double angle;
    public AutoRotateShoulderToAngle(double angle) {
        this.angle = angle;
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setShoulderTargetPos(angle, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.shoulderPidController.atSetpoint();
    }
}
