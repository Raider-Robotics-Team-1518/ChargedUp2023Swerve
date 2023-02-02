package frc.robot.subsystems.moving;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

    private final Solenoid clawSolenoid;

    private boolean open;

    public static ClawSubsystem INSTANCE;
    public ClawSubsystem() {
        INSTANCE = this;

        clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        open = false;
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


    public static void toggleSolenoid() {
        INSTANCE.clawSolenoid.set(!INSTANCE.clawSolenoid.get());
        INSTANCE.open = !INSTANCE.open;
    }

    public static boolean isOpen() {
        return INSTANCE.open;
    }

    public Solenoid getClawSolenoid() {
        return INSTANCE.clawSolenoid;
    }
}
