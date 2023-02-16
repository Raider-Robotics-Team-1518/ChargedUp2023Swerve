package frc.robot.commands.operational.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ArmSetDesiredPoint extends CommandBase {
    /**
     * Creates a new DriveAdjustModuleZeroPoint.
     * 
     * Each module has a different 0 angle. To solve this, 
     * physically angle each module straight. 
     * Then, tell all modules its current position is at 0. 
     * The later is done by the Command 
     * DriveResetAllModulePositionsToZero 
     */ 
    public ArmSetDesiredPoint() {
      addRequirements(RobotContainer.armSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double val = SmartDashboard.getNumber("DesireddPoint", 5);
        RobotContainer.armSubsystem.armPidController.setSetpoint(val);
        RobotContainer.armSubsystem.desiredPoint = val;
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
  }

