package frc.robot.subsystems.moving;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private final TalonSRX clawMotor = new TalonSRX(Constants.ARM_CLAW_ID);
  private final DigitalInput clawMinSwitch = new DigitalInput(Constants.CLAW_MIN_LIMIT);
  private final DigitalInput clawMaxSwitch = new DigitalInput(Constants.CLAW_MAX_LIMIT);


  public static ClawSubsystem INSTANCE;
  public ClawSubsystem() {
      INSTANCE = this;
  }

  public boolean isClawInRange() {
    return !(clawMinSwitch.get() || clawMaxSwitch.get());
  }

  @Override
  public void periodic() {

  }

  public TalonSRX getClawMotor() {
    return this.clawMotor;
  }
}
