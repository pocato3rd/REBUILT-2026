package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

public class Climber {
	private final CANBus canivore = new CANBus("canivore");
	private final TalonFX climbMotor = new TalonFX(18, canivore);
	private final StatusSignal<Angle> climberPosition;
	private final StatusSignal<AngularVelocity> climberVelocity;
	private final MotionMagicTorqueCurrentFOC climbMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final VoltageOut climbMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final Timer homingTimer = new Timer();
	private final double upPosition = 80.0;
	private final double downPosition = 2.0;
	private final double posTol = 1.0;
  public enum Mode {HOME, UP, DOWN}
	public Mode currMode = Mode.HOME;
	private boolean isHomed = false;
	private double desiredPosition = 0.0;

	public Climber() {
		configMotor(climbMotor, false); // Configures the motor with counterclockwise rotation positive.
		climberPosition = climbMotor.getPosition();
		climberVelocity = climbMotor.getVelocity();
		BaseStatusSignal.setUpdateFrequencyForAll(250.0, climberPosition, climberVelocity);
		ParentDevice.optimizeBusUtilizationForAll(climbMotor);
	}

	public void init() {
		homingTimer.restart();
	}

	public void perioidic() {
		switch (currMode) {
			case HOME:
				climbMotor.setControl(climbMotorVoltageRequest.withOutput(-2.0).withEnableFOC(true));
				if (Math.abs(getVelocity()) > 0.5) homingTimer.restart();
				if (homingTimer.get() > 1.0) {
					climbMotor.setPosition(0.0, 0.03);
          isHomed = true;
					currMode = Mode.DOWN;
          desiredPosition = downPosition;
				}
			break;

			case UP:
				climbMotor.setControl(climbMotorPositionRequest.withPosition(upPosition)); // Sets the position of the motor in shaft rotations.
			break;

			case DOWN:
				climbMotor.setControl(climbMotorPositionRequest.withPosition(downPosition)); // Sets the position of the motor in shaft rotations.
			break;
		}
	}

	public void moveUp() {
		if (isHomed) {
			currMode = Mode.UP;
      desiredPosition = upPosition;
		}
	}

	public void moveDown() {
		if (isHomed) {
			currMode = Mode.DOWN;
      desiredPosition = downPosition;
		}
	}

	public Mode getMode() {
		return currMode;
	}

	public boolean atDesiredPosition() {
		return Math.abs(desiredPosition - getPosition()) < posTol;
	}

	public double getPosition() {
		return climberPosition.refresh().getValueAsDouble();
	}

	public double getVelocity() {
		return climberVelocity.refresh().getValueAsDouble();
	}

	public void updateDash() {
		//SmartDashboard.putNumber("Climber Timer", homingTimer.get());
    //SmartDashboard.putBoolean("Climber atDesired position", atDesiredPosition());
		//SmartDashboard.putBoolean("Climber isHomed", isHomed);
		//SmartDashboard.putString("Climber Mode", currMode.toString());
		//SmartDashboard.putNumber("Climber Position", getPosition());
		//SmartDashboard.putNumber("Climber Velocity", getVelocity());
	}
	
	private void configMotor(TalonFX motor, boolean invert) {
		TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

		motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		// MotionMagicTorqueFOC closed-loop control configuration.
		motorConfigs.Slot0.kP = 37.0; // Units: amperes per 1 rotation of error.
		motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
		motorConfigs.Slot0.kD = 0.84; // Units: amperes per 1 rotation / 1 second of error.
		motorConfigs.MotionMagic.MotionMagicAcceleration = 1000.0; // Units: rotations per second per second.
		motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Units: rotations per second.

		motor.getConfigurator().apply(motorConfigs, 0.03);
	}
}