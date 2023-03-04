package frc.robot.subsystems.moving;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClawSubsystem extends SubsystemBase {
  private final TalonSRX clawMotor = new TalonSRX(Constants.ARM_CLAW_ID);

  public static ClawSubsystem INSTANCE;
  public ClawSubsystem() {
      INSTANCE = this;
      clawMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {

  }

  public void enableClawMotor(double speed) {
    RobotContainer.clawSubsystem.getClawMotor().set(ControlMode.PercentOutput, speed);
  }

  public boolean outerSwitchReached() {
    return clawMotor.isFwdLimitSwitchClosed() == 1;
  }

  public boolean innerSwitchReached() {
    return clawMotor.isRevLimitSwitchClosed() == 1;
  }

  public TalonSRX getClawMotor() {
    return this.clawMotor;
  }
}
