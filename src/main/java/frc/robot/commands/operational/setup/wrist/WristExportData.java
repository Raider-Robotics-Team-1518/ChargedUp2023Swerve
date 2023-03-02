package frc.robot.commands.operational.setup.wrist;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.moving.ArmSubsystem.DumpMode;

/*
 * Export Wrist motion data into a csv file that is readable with columns time(ms), input (code input motor speed), output (actual encoder response to our code input)
 * This data can then be used in https://pidtuner.com (after the time column is converted to seconds) in order to tune PID accuratly.
 */
public class WristExportData extends CommandBase {
    public WristExportData() {
      addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
      RobotContainer.armSubsystem.stopArm();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        try {
            RobotContainer.armSubsystem.writer = new BufferedWriter(new FileWriter(RobotContainer.armSubsystem.wristDumpFile));
        } catch (IOException e) {
            e.printStackTrace();
        }
        RobotContainer.armSubsystem.toggleDumping(DumpMode.WRIST);
    }
  
    @Override
    public void end(boolean interrupted) {
        RobotContainer.armSubsystem.setDoneDumping(false);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.getDoneDumping();
    }
}
