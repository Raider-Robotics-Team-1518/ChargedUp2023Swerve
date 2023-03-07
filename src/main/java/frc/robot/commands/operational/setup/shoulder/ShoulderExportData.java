package frc.robot.commands.operational.setup.shoulder;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.moving.ArmSubsystem.DumpMode;

/*
 * Export Shoulder motion data into a csv file that is readable with columns time(ms), input (code input motor speed), output (actual encoder response to our code input)
 * This data can then be used in https://pidtuner.com (after the time column is converted to seconds) in order to tune PID accuratly.
 */
public class ShoulderExportData extends CommandBase {
    public ShoulderExportData() {
      addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
      RobotContainer.armSubsystem.stopArm();
    }
  
    @Override
    public void execute() {
        try {
            RobotContainer.armSubsystem.writer = new BufferedWriter(new FileWriter(RobotContainer.armSubsystem.shoulderDumpFile));
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            RobotContainer.armSubsystem.toggleDumping(DumpMode.SHOULDER);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
  
    @Override
    public void end(boolean interrupted) {
        RobotContainer.armSubsystem.setDoneDumping(false);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
  
    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.getDoneDumping();
    }
}
