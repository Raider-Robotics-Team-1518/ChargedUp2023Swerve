package frc.robot.subsystems.moving;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private final TalonSRX clawMotor = new TalonSRX(Constants.ARM_CLAW_ID);

  public static ClawSubsystem INSTANCE;
  public ClawSubsystem() {
      INSTANCE = this;
  }

  @Override
  public void periodic() {

  }

  public TalonSRX getClawMotor() {
    return this.clawMotor;
  }
}
