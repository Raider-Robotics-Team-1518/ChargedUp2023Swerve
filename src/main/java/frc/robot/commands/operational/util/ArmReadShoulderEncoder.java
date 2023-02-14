package frc.robot.commands.operational.util;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ArmReadShoulderEncoder extends CommandBase {
    public ArmReadShoulderEncoder() {
      addRequirements(RobotContainer.armSubsystem);
    }
    @Override
    public void initialize() {
        RobotContainer.armSubsystem.stopArm();
    }
  
    @Override
    public void execute() {
      SparkMaxAbsoluteEncoder shoulderEncoder = RobotContainer.armSubsystem.getShoulderMotor().getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
      SmartDashboard.putNumber("shoulderAbsValue", shoulderEncoder.getPosition()*shoulderEncoder.getPositionConversionFactor());
    }
  
    @Override
    public void end(boolean interrupted) {
      RobotContainer.armSubsystem.stopArm();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
  
