package frc.robot.subsystems.moving;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // "shoulder" is the pivot point at the top of the tower, raises & lowers the arm
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.ARM_SHOULDER_ID, MotorType.kBrushless);
    // "wrist" is the pivot point at the end of the arm, raises and lowers the claw
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.ARM_WRIST_ID, MotorType.kBrushless);
    // "telescope" is the part of the arm that extends/retracts
    private final CANSparkMax telescopeMotor = new CANSparkMax(Constants.ARM_TELESCOPE_ID, MotorType.kBrushless);

    public static ArmSubsystem INSTANCE;
    public ArmSubsystem() {
        INSTANCE = this;
    }

    /* All of these values should be measured using the Absolute Encoder, not the Relative Encoder, 
    frc.robot.commands.operational.ArmReadShoulderEncoder.java command should display a value in Shuffleboard when enabled */ 
    private final double shoulderHeighestRev = 0.0d; // 180 deg (Facing straight up, this position is very unlikely in real life, but must be measured for angle conversion accuracy)

    private final double shoulderEvenRev = 0.0d; // 90 deg (Facing straight out, this will be pretty close to the angle needed to place a game object)

    private final double shoulderLowestRev = 0.0d; /*0 deg (Facing straight down, this value will also be impossible to reach in real life due to robot parts being in the way,
                                                     but it still must be measured in order to get accurate encoder resolution to angle conversion)*/

    private double currentTargetPos = 0.0d; // this value should be the FRC legal resting position (unknown value resulting in between 0 and 90 degrees)
  
    @Override
    public void periodic() {
        //fixateShoulder(currentTargetPos);
    }

    public CommandBase fixateShoulder(double targetPosition) {
        return Commands.sequence(moveArmBestDirection(targetPosition),
            waitUntilShoulderPosEquals(targetPosition),
            stopShoulder());
    }

    public CommandBase waitUntilShoulderPosEquals(double targetRev) {
        if(shoulderWithinRange(targetRev)) return runOnce(()->{});
        return Commands.waitUntil(() -> shoulderWithinRange(targetRev));
    }

    public CommandBase moveArmBestDirection(double targetRev) {
        SparkMaxAbsoluteEncoder absoluteEncoder = shoulderMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        double currentRevolutions = absoluteEncoder.getPosition()*absoluteEncoder.getPositionConversionFactor();
        if(shoulderWithinRange(targetRev)) return runOnce(()->{});
        if(currentRevolutions < targetRev) return shoulderUp();
        if(currentRevolutions > targetRev) return shoulderDown();
        return runOnce(()->{});
    }

    public boolean shoulderWithinRange(double targetPosition) {
        SparkMaxAbsoluteEncoder absoluteEncoder = shoulderMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        double currentRevolutions = absoluteEncoder.getPosition()*absoluteEncoder.getPositionConversionFactor();
        return (currentRevolutions <= targetPosition+10) && (currentRevolutions >= targetPosition-10);
    }


    public CommandBase shoulderUp() {
        return runOnce(() -> shoulderMotor.set(1.0d));
    }
    

    public CommandBase shoulderDown() {
        return runOnce(() -> shoulderMotor.set(-1.0d));
    }

    public CommandBase rotateWrist(int degrees) {
        return Commands.sequence(resetWristEncoder(),
            enableWrist(),
            waitUntilWristRotatedToDeg(degrees),
            disableWrist());
    }

    public CommandBase telescopeExtend() {
        if(telescopeMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed()) return runOnce(()->{});
        return runOnce(() -> {
            telescopeMotor.set(0.5d);
        });
    }

    public CommandBase telescopeRetract() {
        if(telescopeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed()) return runOnce(()->{});
        return runOnce(() -> telescopeMotor.set(-0.5d));
    }

    public CommandBase waitUntilWristRotatedToDeg(int degrees) {
        return Commands.waitUntil(() -> (wristMotor.getEncoder().getPosition()*360) >= degrees);
    }

    public CommandBase enableWrist() {
        return runOnce(() -> wristMotor.set(1.0d));
    }

    public CommandBase disableWrist() {
        return runOnce(() -> wristMotor.set(1.0d));
    }

    public CommandBase resetWristEncoder() {
        return runOnce(() -> {
            wristMotor.getEncoder().setPosition(0.0d);
        });
    }

    public CommandBase stopShoulder() {
        return runOnce(() -> shoulderMotor.set(0.0d));
    }

    public CommandBase stopWrist() {
        return runOnce(() -> wristMotor.set(0.0d));
    }

    public CommandBase stopTelescope() {
        return runOnce(() -> telescopeMotor.set(0.0d));
    }

    public CommandBase stopArm() {
        return Commands.sequence(stopShoulder(), stopWrist(), stopTelescope());
    }


    public CANSparkMax getShoulderMotor() {
        return INSTANCE.shoulderMotor;
    }

    public CANSparkMax getWristMotor() {
        return INSTANCE.wristMotor;
    }

    public CANSparkMax getTelescopeMotor() {
        return INSTANCE.telescopeMotor;
    }

}
