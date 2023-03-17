package frc.robot.commands.autonomous.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.PlaceMode;

/*
 * Rotate arm to prime dropping position
 */

public class AutoRotateShoulderPlacing extends CommandBase {
    private final PlaceMode placeMode;
    public AutoRotateShoulderPlacing(PlaceMode placeMode) {
        this.placeMode = placeMode;
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setShoulderTargetPos(placeMode.getAngle(), true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.shoulderPidController.atSetpoint();
    }
}
