package frc.robot.subsystems.moving;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private final CANSparkMax clawMotorOne = new CANSparkMax(Constants.ARM_CLAW_ID1, MotorType.kBrushless);
  private final CANSparkMax clawMotorTwo = new CANSparkMax(Constants.ARM_CLAW_ID2, MotorType.kBrushless);
  private final MotorControllerGroup clawMotorGroup;

  public static ClawSubsystem INSTANCE;
  public ClawSubsystem() {
      INSTANCE = this;
      clawMotorOne.setInverted(true);
      clawMotorOne.setIdleMode(IdleMode.kBrake);
      clawMotorTwo.setIdleMode(IdleMode.kBrake);  
      clawMotorGroup = new MotorControllerGroup(clawMotorOne, clawMotorTwo);
    }

  @Override
  public void periodic() {

  }

  public void setClawSpeed(double speed) {
    clawMotorGroup.set(speed);
  }

}
