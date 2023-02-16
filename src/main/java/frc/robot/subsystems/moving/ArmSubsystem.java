package frc.robot.subsystems.moving;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.operational.util.ArmExportData;

public class ArmSubsystem extends SubsystemBase {
    // "shoulder" is the pivot point at the top of the tower, raises & lowers the arm
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.ARM_SHOULDER_ID, MotorType.kBrushless);
    // "wrist" is the pivot point at the end of the arm, raises and lowers the claw
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.ARM_WRIST_ID, MotorType.kBrushless);
    // "telescope" is the part of the arm that extends/retracts
    private final CANSparkMax telescopeMotor = new CANSparkMax(Constants.ARM_TELESCOPE_ID, MotorType.kBrushless);


    public final PIDController armPidController = new PIDController(Constants.ARM_MOVE_P, Constants.ARM_MOVE_I, Constants.ARM_MOVE_D);


    /* All of these values should be measured using the Absolute Encoder, not the Relative Encoder, 
    frc.robot.commands.operational.ArmReadShoulderEncoder.java command should display a value in Shuffleboard when enabled */ 
    private final double shoulderHeighestRev = 0.0d; // 180 deg (Facing straight up, this position is very unlikely in real life, but must be measured for angle conversion accuracy)

    private final double shoulderEvenRev = 0.0d; // 90 deg (Facing straight out, this will be pretty close to the angle needed to place a game object)

    private final double shoulderLowestRev = 0.0d; /*0 deg (Facing straight down, this value will also be impossible to reach in real life due to robot parts being in the way,
                                                     but it still must be measured in order to get accurate encoder resolution to angle conversion)*/

    private double currentTargetPos = 0.0d; // this value should be the FRC legal resting position (unknown value resulting in between 0 and 90 degrees)


    public double desiredPoint = 8;
    private boolean isDumping = false;
    private boolean doneDumping = false;
    private String dumpFile = "/home/lvuser/pid_data.csv";
    private BufferedWriter writer;
    private long start = -1L;

    public static ArmSubsystem INSTANCE;
    public ArmSubsystem() {
        INSTANCE = this;
        try { 
            new File(dumpFile).createNewFile();
            writer = new BufferedWriter(new FileWriter(dumpFile)); 
        } catch(Exception e){ 
            e.printStackTrace(); 
        }
        start = -1L;

        shoulderMotor.setIdleMode(IdleMode.kBrake);
        armPidController.setTolerance(0.1);
        armPidController.setSetpoint(desiredPoint);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("ShoulderAbsEncVal", getArmAbsolutePosition());
        SmartDashboard.putNumber("DesiredValue", desiredPoint);

        // PID Data dumping
        if(isDumping) {
            try {
                if(start == -1L) {
                    start = System.currentTimeMillis();
                    writer.append("Time,Input,Output");
                    writer.newLine();
                }
                doDump();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if(!DriverStation.isEnabled()) return;
        /*!!!!!!!!!!!!!!!!! Enable-Require Tasks !!!!!!!!!!!!!!!!!!!!*/

        //System.out.println("At Setpoint: " + (armPidController.atSetpoint() ? "True" : "False"));
        if(!armPidController.atSetpoint()) {
            //System.out.println("Sent Input: " + armPidController.calculate(this.getArmAbsolutePosition(), 5));
            shoulderMotor.set(armPidController.calculate(this.getArmAbsolutePosition(), desiredPoint));
        } else {
            shoulderMotor.set(0);
        }
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
        return runOnce(() -> telescopeMotor.set(0.25d));
    }

    public CommandBase telescopeRetract() {
        return runOnce(() -> telescopeMotor.set(-0.25d));
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

    public void toggleDumping() {
        INSTANCE.isDumping = !INSTANCE.isDumping;
    }

    public void resetArmAbsolutePosition() {
        shoulderMotor.getEncoder().setPosition(0.0);
    }

    public double getArmAbsolutePosition() {
        return shoulderMotor.getEncoder().getPosition();
    }

    public void doDump() throws IOException {
        float timeElapsed = (System.currentTimeMillis() - start) ; // time elapsed in seconds
        if(timeElapsed >= 1000 && timeElapsed <= 2000) {
            shoulderMotor.set(0.1);
        } else if(timeElapsed > 2000 && timeElapsed <= 3000) {
            shoulderMotor.set(0);
        } else if(timeElapsed > 3000 && timeElapsed <= 3500) {
            shoulderMotor.set(-0.1);
        } else if(timeElapsed > 3500 && timeElapsed <= 4500) {
            shoulderMotor.set(0);
        } else if(timeElapsed > 4500 && timeElapsed <= 5500) {
            shoulderMotor.set(0.2);
        } else if(timeElapsed > 5500 && timeElapsed <= 6500) {
            shoulderMotor.set(0);
        } else if(timeElapsed > 6500 && timeElapsed <= 7000) {
            shoulderMotor.set(-0.2);
        } else if(timeElapsed > 7000 && timeElapsed <= 8000) {
            shoulderMotor.set(0.0);
        }

        writer.append(timeElapsed + "," + shoulderMotor.get() + "," + getArmAbsolutePosition());
        writer.newLine();
        if(timeElapsed >= 8000) {
            isDumping = false;
            doneDumping = true;
            start = -1L;
            writer.close();
        }
    }

    public boolean getDumping() {
        return INSTANCE.isDumping;
    }

    public void setDoneDumping(boolean b) {
        INSTANCE.doneDumping = b;
    }
    
    public boolean getDoneDumping() {
        return INSTANCE.doneDumping;
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
