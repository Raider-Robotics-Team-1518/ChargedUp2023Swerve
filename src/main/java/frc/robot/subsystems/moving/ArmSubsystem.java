package frc.robot.subsystems.moving;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    /*
     * Moving Parts
     * !!!! IMPORTANT: SETUP of these VARIABLES using the designated SETUP COMMANDS is REQUIRED before BOT USAGE !!!!!
     */

    // Shoulder
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.ARM_SHOULDER_ID, MotorType.kBrushless);
    public final PIDController shoulderPidController = new PIDController(Constants.ARM_SHOULDER_P, Constants.ARM_SHOULDER_I, Constants.ARM_SHOULDER_D);
    public double maxShoulderPos = Constants.getDouble(Constants.SHOULDER_MAX_POS); // Arm is facing the ceiling (180 deg)
    public double horizontalShoulderPos = Constants.getDouble(Constants.SHOULDER_LEVEL_POS); // 90 deg
    public double minShoulderPos = Constants.getDouble(Constants.SHOULDER_MIN_POS); // As retracted as we can get without breaking electronics (unknown angle)
    public double idleShoulderAngle = Constants.getDouble(Constants.SHOULDER_IDLE_ANGLE); // (degrees)

    // Wrist
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.ARM_WRIST_ID, MotorType.kBrushless);
    public final PIDController wristPidController = new PIDController(Constants.ARM_WRIST_P, Constants.ARM_WRIST_I, Constants.ARM_WRIST_D);
    public double maxWristPos = Constants.getDouble(Constants.WRIST_MAX_POS); // 90 deg (Facing up)
    public double minWristPos = Constants.getDouble(Constants.WRIST_MIN_POS); // 0 deg (Level)
    public double idleWristAngle = Constants.getDouble(Constants.WRIST_IDLE_ANGLE); // (degrees)
    public boolean lockedWrist = true;

    // Telescope
    private final CANSparkMax telescopeMotor = new CANSparkMax(Constants.ARM_TELESCOPE_ID, MotorType.kBrushless);
    public double minTelescopePos = Constants.getDouble(Constants.TELESCOPE_MIN_POS); // Telescope fully retracted
    public double maxTelescopePos = Constants.getDouble(Constants.TELESCOPE_MAX_POS); // Telescope fully extended
    public double desiredTelescopeSpeed = 0.0d;

    // Objects specific to the dump process
    private boolean isDumping = false;
    private boolean doneDumping = false;
    private DumpMode currentDumpMode = DumpMode.WRIST;
    public String shoulderDumpFile = "/home/lvuser/motion/shoulder_motion_data.csv";
    public String wristDumpFile = "/home/lvuser/motion/wirst_motion_data.csv";
    public BufferedWriter writer;
    private long start = -1L;

    public static ArmSubsystem INSTANCE;
    public ArmSubsystem() {
        INSTANCE = this;

        setupDumping();
        setArmBrake();

        // shoulder pid setup
        setShoulderTargetPos(idleShoulderAngle, true);
        shoulderPidController.setTolerance(0.1);

        // wrist pid setup
        setWristTargetPos(idleWristAngle, true);
        wristPidController.setTolerance(0.1);
    }

    @Override
    public void periodic() {

        // PID Data dumping
        if(isDumping) {
            try {
                if(start == -1L) {
                    start = System.currentTimeMillis();
                    writer.append("Time,Input,Output");
                    writer.newLine();
                }
                switch(currentDumpMode) {
                    case SHOULDER:
                        doShoulderDump();
                        break;
                    case WRIST:
                        doWristDump();
                        break;
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        // SetupState Dashboard Information
        if(Constants.setupState) {
            SmartDashboard.putBoolean("ShlderFwdLimit", getShoulderMotor().getForwardLimitSwitch(Type.kNormallyOpen).isPressed());
            SmartDashboard.putBoolean("ShlderRevLimit", getShoulderMotor().getReverseLimitSwitch(Type.kNormallyOpen).isPressed());
            SmartDashboard.putNumber("ShoulderEncoderPos", getShoulderPosition());
            SmartDashboard.putNumber("ShoulderAngle", getShoulderAngle());
            SmartDashboard.putNumber("WristEncoderPos", getWristPosition());
            SmartDashboard.putNumber("WristAngle", getWristAngle());
            SmartDashboard.putNumber("TelescopePos", getTelescopePosition());
            return;
        }

        if(!DriverStation.isEnabled()) {
            return;
        }

        telescopeArbitraryFF();

        // debugging 
        /*
        if(!DriverStation.isEnabled()) return;
        !!!!!!!!!!!!!!!!! Enable-Require Tasks !!!!!!!!!!!!!!!!!!!!
        //fixateShoulder(); fixate just shoulder motor
        //fixateWrist(); fixate just wrist motor
        fixateArm(INSTANCE.lockedWrist); // fixate both
        */
    }

    public void setArmCoast() {
        shoulderMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setIdleMode(IdleMode.kCoast);
        telescopeMotor.setIdleMode(IdleMode.kBrake); // Should never have to leave brake mode for telescope
    }

    public void setArmBrake() {
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);
        telescopeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void overcomeGravityTelescope() {
        // basic equation assuming the cosine of the shoulder angle is porportionate to motor input speed against gravity (abitrary feed forward)

    }

    public void setTelescopeSpeed(double speed) {
        this.desiredTelescopeSpeed = speed;
    }

    public void telescopeArbitraryFF() {
        /*if(this.desiredTelescopeSpeed != 0.0d) {
            telescopeMotor.set(this.desiredTelescopeSpeed);
            return;
        }
        telescopeMotor.set((this.desiredTelescopeSpeed + (Constants.ARM_TELESCOPE_GRAVITY_FACTOR * Math.cos(getShoulderAngle()))));*/
        telescopeMotor.set(this.desiredTelescopeSpeed);
    }

    public void setupDumping() {
        try { 
            new File("/home/lvuser/motion").mkdir();
            new File(shoulderDumpFile).createNewFile();
            new File(wristDumpFile).createNewFile();
            writer = new BufferedWriter(new FileWriter(shoulderDumpFile)); 
        } catch(Exception e){ 
            e.printStackTrace(); 
        }
        start = -1L;

    }

    public void setWristTargetPos(double target, boolean angle) {
        double targetPos = target;
        if(angle) targetPos = ((target/90)*maxWristPos);
        wristPidController.setSetpoint(targetPos);
    }

    public void setShoulderTargetPos(double target, boolean angle) {
        double targetPos = target;
        if(angle) {
            if(target > 90) {
                targetPos = ((target-90)/90)*maxShoulderPos;
            } else {
                targetPos = ((target)/90)*-maxShoulderPos;
            }
        }
        shoulderPidController.setSetpoint(targetPos);
    }

    public void fixateArm(boolean lockedWrist) {
        if(lockedWrist) {
            setWristTargetPos(90+getShoulderAngle(), true);
        }
        fixateShoulder();
    }

    public void fixateWrist() {
        if(!wristPidController.atSetpoint()) {
            wristMotor.set(wristPidController.calculate(this.getWristPosition(), wristPidController.getSetpoint()));
        } else {
            wristMotor.set(0);
        }
    }

    public void fixateShoulder() {
        if(!shoulderPidController.atSetpoint()) {
            shoulderMotor.set(shoulderPidController.calculate(this.getShoulderPosition(), shoulderPidController.getSetpoint()));
        } else {
            shoulderMotor.set(0);
        }
    }

    public boolean isWristInRange() {
        double wristPos = wristMotor.getEncoder().getPosition();
        return ((wristPos < maxWristPos) && (wristPos > minWristPos));
    }

    public boolean isTelescopeInRange() {
        double telescopePos = telescopeMotor.getEncoder().getPosition();
        return ((telescopePos < maxTelescopePos) && (telescopePos > minTelescopePos));
    }

    public double getWristAngle() {
        return ((getWristPosition()/maxWristPos)*180);
    }


    public double getShoulderAngle() {
        /* since i only know the position from (zero/90 deg) horizontal to (180 deg) the highest point, 
        calculations must be done to account for negative values */
        if(getShoulderPosition() > 0) {
            // above (zero) horizontal pos
            return 90+((Math.abs(getShoulderPosition()) / maxShoulderPos)*90);
        }
        return 90-((getShoulderPosition() / (-1*maxShoulderPos)) * 90);
    }

    public void resetWristPosition() {
        wristMotor.getEncoder().setPosition(0.0d);
    }

    public double getWristPosition() {
        return wristMotor.getEncoder().getPosition();
    }


    public double getTelescopePosition() {
        return telescopeMotor.getEncoder().getPosition();
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

    public void toggleDumping(DumpMode dumpMode) {
        INSTANCE.currentDumpMode = dumpMode;
        INSTANCE.isDumping = !INSTANCE.isDumping;
    }

    public void setShoulderMaxPos(double highestPosition) {
        INSTANCE.maxShoulderPos = highestPosition;
    }

    public void resetTelescopePosition() {
        telescopeMotor.getEncoder().setPosition(0.0);
    }

    public void resetShoulderPosition() {
        shoulderMotor.getEncoder().setPosition(0.0);
    }

    public double getShoulderPosition() {
        return shoulderMotor.getEncoder().getPosition();
    }

    public void doShoulderDump() throws IOException {
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

        writer.append(timeElapsed + "," + shoulderMotor.get() + "," + getShoulderPosition());
        writer.newLine();
        if(timeElapsed >= 8000) {
            isDumping = false;
            doneDumping = true;
            start = -1L;
            writer.close();
        }
    }

    public void doWristDump() throws IOException {
        float timeElapsed = (System.currentTimeMillis() - start) ; // time elapsed in seconds
        if(timeElapsed >= 1000 && timeElapsed <= 2000) {
            wristMotor.set(0.2);
        } else if(timeElapsed > 2000 && timeElapsed <= 3000) {
            wristMotor.set(0);
        } else if(timeElapsed > 3000 && timeElapsed <= 3500) {
            wristMotor.set(-0.2);
        } else if(timeElapsed > 3500 && timeElapsed <= 4500) {
            wristMotor.set(0);
        } else if(timeElapsed > 4500 && timeElapsed <= 5500) {
            wristMotor.set(0.4);
        } else if(timeElapsed > 5500 && timeElapsed <= 6500) {
            wristMotor.set(0);
        } else if(timeElapsed > 6500 && timeElapsed <= 7000) {
            wristMotor.set(-0.4);
        } else if(timeElapsed > 7000 && timeElapsed <= 8000) {
            wristMotor.set(0.0);
        }

        writer.append(timeElapsed + "," + shoulderMotor.get() + "," + getWristPosition());
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

    public static enum DumpMode {
        SHOULDER, WRIST
    }

}
