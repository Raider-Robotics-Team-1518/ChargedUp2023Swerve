package frc.robot.subsystems.moving;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.ARM_SHOULDER_ID, MotorType.kBrushless);

    private final CANSparkMax wristMotor = new CANSparkMax(Constants.ARM_WRIST_ID, MotorType.kBrushless);

    private final CANSparkMax telescopeMotor = new CANSparkMax(Constants.ARM_TELESCOPE_ID, MotorType.kBrushless);

    public static ArmSubsystem INSTANCE;
    public ArmSubsystem() {
        INSTANCE = this;
    }
  

    public CommandBase exampleMethodCommand() {
      // Inline construction of command goes here.
      // Subsystem::RunOnce implicitly requires `this` subsystem.
      return runOnce(
          () -> {
            /* one-time action goes here */
          });
    }

    public boolean exampleCondition() {
      return false;
    }
  
    @Override
    public void periodic() {

    }


    public void toggleTelescope(boolean extend) {
        telescopeMotor.set(extend ? 1 : -1);
    }


    public static CANSparkMax getShoulderMotor() {
        return INSTANCE.shoulderMotor;
    }

    public static CANSparkMax getWristMotor() {
        return INSTANCE.wristMotor;
    }

    public static CANSparkMax getTelescopeMotor() {
        return INSTANCE.telescopeMotor;
    }
}
