package frc.robot.commands.operational.setup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.moving.ArmSubsystem.DumpMode;

public class ShoulderExportData extends CommandBase {
    /**
     * Creates a new DriveAdjustModuleZeroPoint.
     * 
     * Each module has a different 0 angle. To solve this, 
     * physically angle each module straight. 
     * Then, tell all modules its current position is at 0. 
     * The later is done by the Command 
     * DriveResetAllModulePositionsToZero 
     */ 
    public ShoulderExportData() {
      addRequirements(RobotContainer.armSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      RobotContainer.armSubsystem.stopArm();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.armSubsystem.toggleDumping(DumpMode.SHOULDER);
    }
  
    // Called once the command ends or is interrupted.
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
