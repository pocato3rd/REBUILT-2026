package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class Intake {  
  // Motors and Sensors
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX rightArmMotor = new TalonFX(16, canivore);
  private final TalonFXS rightRollerMotor = new TalonFXS(15, canivore);
  private final TalonFX rightCenteringMotor = new TalonFX(20, canivore);
  private final TalonFX leftArmMotor = new TalonFX(14, canivore);
  private final TalonFXS leftRollerMotor = new TalonFXS(17, canivore);
  private final TalonFX leftCenteringMotor = new TalonFX(13, canivore);
  private final DutyCycleEncoder leftArmEncoder = new DutyCycleEncoder(0);
  private final DutyCycleEncoder rightArmEncoder = new DutyCycleEncoder(1);
  
  // Control Requests
  private final MotionMagicTorqueCurrentFOC rightArmMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC leftArmMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final VoltageOut rightArmMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut leftArmMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut rightRollerMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut leftRollerMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut rightCenteringMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut leftCenteringMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  // Status Signals
  private final StatusSignal<Angle> leftArmPosition;
	private final StatusSignal<AngularVelocity> leftArmVelocity;
  private final StatusSignal<Angle> rightArmPosition;
	private final StatusSignal<AngularVelocity> rightArmVelocity;

  private final Timer leftHomingTimer = new Timer();
  private final Timer rightHomingTimer = new Timer();
  private final double armPosTol = 0.5;
  private final double armIntakePosition = 9.9; // Can adjust
  private final double armStowPosition = 0.2; // Can adjust
  public enum Mode {HOME, LEFT, RIGHT, STOW}
  private Mode currMode = Mode.HOME;
  private boolean leftArmIsHomed = false;
  private boolean rightArmIsHomed = false;
  private double desiredLeftArmPosition = 0.0;
  private double desiredRightArmPosition = 0.0;
  private boolean leftArmIsStowed = true;
  private boolean rightArmIsStowed = true;

  public Intake() {
    configArmMotor(leftArmMotor, true);
    configArmMotor(rightArmMotor, false);
    configRollerMotor(leftRollerMotor, false);
    configRollerMotor(rightRollerMotor, true);
    configCenteringMotor(leftCenteringMotor, true);
    configCenteringMotor(rightCenteringMotor, false);
    leftArmPosition = leftArmMotor.getPosition();
    leftArmVelocity = leftArmMotor.getVelocity();
    rightArmPosition = rightArmMotor.getPosition();
    rightArmVelocity = rightArmMotor.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, leftArmPosition, leftArmVelocity, rightArmPosition, rightArmVelocity);
		ParentDevice.optimizeBusUtilizationForAll(rightArmMotor, rightRollerMotor, rightCenteringMotor, leftArmMotor, leftRollerMotor, leftCenteringMotor);
  }

  public void init() {
    leftHomingTimer.restart();
    rightHomingTimer.restart();
  }

  public void periodic() {
    switch (currMode) {
      case HOME:
        leftArmMotor.setControl(leftArmMotorVoltageRequest.withOutput(-1.0).withEnableFOC(true));
        rightArmMotor.setControl(rightArmMotorVoltageRequest.withOutput(-1.0).withEnableFOC(true));

        if (Math.abs(getLeftArmVelocity()) > 0.5) leftHomingTimer.restart();
        if (Math.abs(getRightArmVelocity()) > 0.5) rightHomingTimer.restart();

        if (leftHomingTimer.get() > 1.0 && !leftArmIsHomed) {
          leftArmMotor.setPosition(0.0, 0.03);
          leftArmIsHomed = true;
          leftArmIsStowed = true;
        }
        if (rightHomingTimer.get() > 1.0 && !rightArmIsHomed) {
          rightArmMotor.setPosition(0.0, 0.03);
          rightArmIsHomed = true;
          rightArmIsStowed = true;
        }

        if (leftArmIsHomed && rightArmIsHomed) {
          currMode = Mode.STOW;
          desiredLeftArmPosition = armStowPosition;
          desiredRightArmPosition = armStowPosition;
        }
      break;

      case LEFT:
        if (leftArmInPosition()) {
          leftArmIsStowed = false;
        }
        if (rightArmInPosition()) {
          rightArmIsStowed = true;
        }
        rightArmMotor.setControl(rightArmMotorPositionRequest.withPosition(armStowPosition));
        if (rightArmIsStowed) {
          leftArmMotor.setControl(leftArmMotorPositionRequest.withPosition(armIntakePosition));
        } else {
          leftArmMotor.setControl(leftArmMotorPositionRequest.withPosition(armStowPosition));
        }
      break;

      case RIGHT:
        if (leftArmInPosition()) {
          leftArmIsStowed = true;
        }
        if (rightArmInPosition()) {
          rightArmIsStowed = false;
        }
        leftArmMotor.setControl(leftArmMotorPositionRequest.withPosition(armStowPosition));
        if (leftArmIsStowed) {
          rightArmMotor.setControl(rightArmMotorPositionRequest.withPosition(armIntakePosition));
        } else {
          rightArmMotor.setControl(rightArmMotorPositionRequest.withPosition(armStowPosition));
        }
      break;

      case STOW:
        if (leftArmInPosition()) {
          leftArmIsStowed = true;
        }
        if (rightArmInPosition()) {
          rightArmIsStowed = true;
        }
        leftArmMotor.setControl(leftArmMotorPositionRequest.withPosition(armStowPosition));
        rightArmMotor.setControl(rightArmMotorPositionRequest.withPosition(armStowPosition));
      break;
    }
    if (leftArmIsStowed) {
      leftRollerMotor.setControl(leftRollerMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
      leftCenteringMotor.setControl(leftCenteringMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
    } else {
      leftRollerMotor.setControl(leftRollerMotorVoltageRequest.withOutput(8.0).withEnableFOC(true));
      leftCenteringMotor.setControl(leftCenteringMotorVoltageRequest.withOutput(4.0).withEnableFOC(true));
    }
    if (rightArmIsStowed) {
      rightRollerMotor.setControl(rightRollerMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
      rightCenteringMotor.setControl(rightCenteringMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
    } else {
      rightRollerMotor.setControl(rightRollerMotorVoltageRequest.withOutput(8.0).withEnableFOC(true));
      rightCenteringMotor.setControl(rightCenteringMotorVoltageRequest.withOutput(4.0).withEnableFOC(true));
    }
  }

  public void leftIntake() {
    if (getMode() != Mode.HOME) {
      currMode = Mode.LEFT;
      desiredLeftArmPosition = armIntakePosition;
      desiredRightArmPosition = armStowPosition;
      leftArmIsStowed = !leftArmInPosition();
      rightArmIsStowed = rightArmInPosition();
    }
  }

  public void rightIntake() {
    if (getMode() != Mode.HOME) {
      currMode = Mode.RIGHT;
      desiredLeftArmPosition = armStowPosition;
      desiredRightArmPosition = armIntakePosition;
      leftArmIsStowed = leftArmInPosition();
      rightArmIsStowed = !rightArmInPosition();
    }
  }

  public void stow() {
    if (getMode() != Mode.HOME) {
      currMode = Mode.STOW;
      desiredLeftArmPosition = armStowPosition;
      desiredRightArmPosition = armStowPosition;
      leftArmIsStowed = leftArmInPosition();
      rightArmIsStowed = rightArmInPosition();
    }
  }

  public Mode getMode() {
    return currMode;
  }

  public double getLeftArmEncoder() {
    return leftArmEncoder.get();
  }

  public double getRightArmEncoder() {
    return rightArmEncoder.get();
  }

  public double getLeftArmPosition() {
    return leftArmPosition.refresh().getValueAsDouble();
  }

  public double getLeftArmVelocity() {
    return leftArmVelocity.refresh().getValueAsDouble();
  }

  public double getLeftArmDesiredPosition() {
    return desiredLeftArmPosition;
  }

  public boolean leftArmInPosition() {
    return Math.abs(getLeftArmPosition() - desiredLeftArmPosition) < armPosTol;
  }

  public double getRightArmPosition() {
    return rightArmPosition.refresh().getValueAsDouble();
  }

  public double getRightArmVelocity() {
    return rightArmVelocity.refresh().getValueAsDouble();
  }

  public double getRightArmDesiredPosition() {
    return desiredRightArmPosition;
  }
  
  public boolean rightArmInPosition() {
    return Math.abs(getRightArmPosition() - desiredRightArmPosition) < armPosTol;
  }

  public boolean isReady() {
    if (getMode() == Mode.LEFT || getMode() == Mode.RIGHT) {
      return leftArmInPosition() && rightArmInPosition();
    } else {
      return false;
    }
  }

  public void updateDash() {
    //SmartDashboard.putString("Intake getMode", currMode.toString());
    //SmartDashboard.putNumber("Intake getLeftArmEncoder", getLeftArmEncoder());
    //SmartDashboard.putNumber("Intake getRightArmEncoder", getRightArmEncoder());
    //SmartDashboard.putNumber("Intake getLeftArmPosition", getLeftArmPosition());
    //SmartDashboard.putNumber("Intake getLeftArmVelocity", getLeftArmVelocity());
    //SmartDashboard.putNumber("Intake getLeftArmDesiredPosition", getLeftArmDesiredPosition());
    //SmartDashboard.putBoolean("Intake leftArmInPosition", leftArmInPosition());
    //SmartDashboard.putNumber("Intake getRightArmPosition", getRightArmPosition());
    //SmartDashboard.putNumber("Intake getRightArmVelocity", getRightArmVelocity());
    //SmartDashboard.putNumber("Intake getRightArmDesiredPosition", getRightArmDesiredPosition());
    //SmartDashboard.putBoolean("Intake rightArmInPosition", rightArmInPosition());
    //SmartDashboard.putBoolean("Intake isReady", isReady());
  }
  
  private void configCenteringMotor(TalonFX motor, boolean invert) {
		TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

		motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }

  private void configRollerMotor(TalonFXS motor, boolean invert) {
		TalonFXSConfiguration motorConfigs = new TalonFXSConfiguration();

    motorConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    motorConfigs.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;

		motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
  
  private void configArmMotor(TalonFX motor, boolean invert) {
		TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

		motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		// MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0/18.75; // Units: amperes per 1 rotation of error.
		motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
		motorConfigs.Slot0.kD = 18.0/18.75; // Units: amperes per 1 rotation / 1 second of error.
		motorConfigs.MotionMagic.MotionMagicAcceleration = 2.0*5800.0/60.0; // Units: rotations per second per second.
		motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/60.0; // Units: rotations per second.


		motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}