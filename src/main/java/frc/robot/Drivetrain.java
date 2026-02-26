package frc.robot;

import java.util.ArrayList;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers.PoseEstimate;

class Drivetrain {
  public static final double maxAcc = 1.0*9.80665/Math.sqrt(2.0); // The maximum acceleration of the robot, typically limited by the coefficient of friction between the swerve wheels and the field.
  public static final double wheelbaseX = (24.0-2*2.625)*0.0254; // The length of the robot from front to back in units of meters. Measured from the centers of each swerve wheel.
  public static final double wheelbaseY = (24.0-2*2.625)*0.0254; // The length of the robot from left to right in units of meters. Measured from the centers of each swerve wheel.
  public static final double wheelbaseR = Math.sqrt(Math.pow(wheelbaseX/2.0, 2) + Math.pow(wheelbaseY/2.0, 2)); // The "radius" of the robot from robot center to the center of the swerve wheel in units of meters.
  public static final double fieldWidth = 8.0137; // The X width of the field in meters. Used to translate between Blue and Red coordinate systems.
  public static final double fieldLength = 651.22*0.0254; // The Y length of the field in meters.
  public static final double maxVelTeleop = SwerveModule.maxVel; // User defined maximum speed of the robot. Enforced during teleop. Unit: meters per second Robot maximum is 4 m/s.
  public static final double maxAngVelTeleop = SwerveModule.maxVel/wheelbaseR; // User defined maximum rotational speed of the robot. Enforced during teleop. Unit: raidans per second Robot maximum is 4pi rad/s.
  public static final double maxAccTeleop = maxAcc; // User defined maximum acceleration of the robot. Enforced during teleop. Unit: meters per second^2 Robot maximum is 5 m/s2.
  public static final double maxAngAccTeleop = maxAccTeleop/wheelbaseR; // User defined maximum rotational acceleration of the robot. Enforced during teleop. Unit: raidans per second^2 Robot maximum is 5pi rad/s2.
  public static final double maxVelAuto = SwerveModule.maxVel; // User defined maximum speed of the robot. Enforced during auto. Unit: meters per second
  public static final double maxAngVelAuto = SwerveModule.maxVel/wheelbaseR; // User defined maximum rotational speed of the robot. Enforced during auto. Unit: raidans per second
  public static final double maxAccAuto = 0.8*maxAcc; // User defined maximum acceleration of the robot. Enforced during auto. Unit: meters per second^2
  public static final double maxAngAccAuto = maxAccAuto/wheelbaseR; // User defined maximum rotational acceleration of the robot. Enforced during auto. Unit: raidans per second^2

  // Positions of the swerve modules relative to the center of the roboot. +x points towards the robot's front. +y points to the robot's left. Units: meters.
  private static final Translation2d frontLeftModulePos = new Translation2d(wheelbaseX/2.0, wheelbaseY/2.0);
  private static final Translation2d frontRightModulePos = new Translation2d(wheelbaseX/2.0, -wheelbaseY/2.0); 
  private static final Translation2d backRightModulePos = new Translation2d(-wheelbaseX/2.0, -wheelbaseY/2.0);
  private static final Translation2d backLeftModulePos = new Translation2d(-wheelbaseX/2.0, wheelbaseY/2.0);
  private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftModulePos, frontRightModulePos, backRightModulePos, backLeftModulePos);

  // Initializes each swerve module.
  private final SwerveModule frontLeftModule = new SwerveModule(1, 2, 1, false, -0.161865, "canivore"); 
  private final SwerveModule frontRightModule = new SwerveModule(3, 4, 2, true, 0.103760 , "canivore");
  private final SwerveModule backRightModule = new SwerveModule(5, 6, 3, true, 0.159912 , "canivore");
  private final SwerveModule backLeftModule = new SwerveModule(7, 8, 4, false, 0.229248, "canivore");
  private final SwerveModule[] modules = {frontLeftModule, frontRightModule, backRightModule, backLeftModule};
  private SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, new Rotation2d()));
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  private final CANBus canivore = new CANBus("canivore"); // Refers to the CAN bus associated with the CANivore.
  private final Pigeon2 pigeon = new Pigeon2(0, canivore); // Pigeon 2.0 CAN Gyroscope
  private final StatusSignal<Angle> pigeonYaw; // Stores the yaw angle measured by the pigeon. 
  private final StatusSignal<Angle> pigeonPitch; // Stores the pitch angle measured by the pigeon.
  private final StatusSignal<Angle> pigeonRoll; // Stores the roll angle measured by the pigeon.
  private final StatusSignal<AngularVelocity> pigeonYawRate; // Stores the yaw velocity measured by the pigeon.
  private final StatusSignal<AngularVelocity> pigeonPitchRate; // Stores the pitch velocity measured by the pigeon.
  private final StatusSignal<AngularVelocity> pigeonRollRate; // Stores the roll velocity measured by the pigeon.

  // Limelight Variables
  public final String[] limelights = {"limelight-shooter", "limelight-backleft", "limelight-backright"}; // Stores the names of all limelights on the robot.
  private final int maxCalibrationFrames = 50; // The number of LL frames that will be averaged to determine the position of the robot when it is disabled() or being calibrated.
  private final int minCalibrationFrames = 3; // The minimum amount of LL frames that must be processed to accept a calibration.
  private double[][] calibrationArray = new double[3][maxCalibrationFrames]; // An array that stores the LL botpose for the most recent frames, up to the number of frames specified by maxCalibrationFrames
  private int calibrationIndex = 0; // The index of the most recent entry into the calibrationPosition array. The index begins at 0 and goes up to calibrationFrames-1, after which it returns to 0 and repeats.
  public int calibrationFrames = 0; // The current number of frames stored in the calibrationPosition array. 
  private final Timer calibrationTimer = new Timer(); // Keeps track of how long it has been since the robot's position has been updated using vision.
  private final Timer accurateCalibrationTimer = new Timer(); // Keeps track of how long it has been since the robot's position has been updated using a highly trusted vision reading.
  private int priorityLimelightIndex = 0; // Stores the index of the most reliable limelight camera based on the number and size of the April Tags visible.

  // Path Following and Targeting Variables
  private ArrayList<PathPlannerTrajectory> paths = new ArrayList<PathPlannerTrajectory>(); // Stores the trajectories generated by Path Planner.
  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), getModulePositions(), new Pose2d(), VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(0.2)), VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE)); // Uses the limelight, motor encoders, and gyroscope to track the position of the robot on the field.
  private final Timer pathTimer = new Timer(); // Keeps track of how long the robot has been following a path. Used to sample Path Planner trajectories.
  private final ProfiledPIDController xDriveController = new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(maxVelAuto, maxAccAuto)); // Controls the x-position of the robot.
  private final ProfiledPIDController yDriveController = new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(maxVelAuto, maxAccAuto)); // Controls the y-position of the robot.
  private final ProfiledPIDController angleDriveController = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(maxAngVelAuto, maxAngAccAuto)); // Controls the angle of the robot.
  private final PIDController xPathController = new PIDController(1.5, 0.0, 0.0);
  private final PIDController yPathController = new PIDController(1.5, 0.0, 0.0);
  private final PIDController anglePathController = new PIDController(1.5, 0.0, 0.0);
  private boolean atDriveGoal = false; // Whether the robot is at the target within the tolerance specified by posTol and angTol when controlled by aimDrive() or moveToTarget()
  private double posTol = 0.02; // The allowable error in the x and y position of the robot in meters.
  private double angTol = 0.6; // The allowable error in the angle of the robot in degrees.
  
  // These variables are updated each period so they can be passed along to the user or the dashboard.
  private double xVel = 0.0; // Unit: meters per second
  private double yVel = 0.0; // Unit: meters per second
  private double angVel = 0.0; // Unit: degrees per second
  private double pathXPos = 0.0; // Unit: meters
  private double pathYPos = 0.0; // Unit: meters
  private double pathAngPos = 0.0; // Unit degrees

  public Drivetrain() {
    pigeon.setYaw(0.0); // Sets the gyro angle to 0 based on the current heading of the robot.
    pigeonYaw = pigeon.getYaw();
    pigeonPitch = pigeon.getPitch();
    pigeonRoll = pigeon.getRoll();
    pigeonYawRate = pigeon.getAngularVelocityZWorld();
    pigeonPitchRate = pigeon.getAngularVelocityYWorld();
    pigeonRollRate = pigeon.getAngularVelocityXWorld();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, pigeonYaw, pigeonYawRate, pigeonPitch, pigeonPitchRate, pigeonRoll, pigeonRollRate);
    ParentDevice.optimizeBusUtilizationForAll(pigeon);
    calibrationTimer.restart();
    accurateCalibrationTimer.restart();
    xDriveController.setIntegratorRange(-maxVelAuto*0.8, maxVelAuto*0.8);
    yDriveController.setIntegratorRange(-maxVelAuto*0.8, maxVelAuto*0.8);
    angleDriveController.setIntegratorRange(-maxAngVelAuto*0.8, maxAngVelAuto*0.8);
    xPathController.setIntegratorRange(-maxVelAuto*0.8, maxVelAuto*0.8);
    yPathController.setIntegratorRange(-maxVelAuto*0.8, maxVelAuto*0.8);
    anglePathController.setIntegratorRange(-maxAngVelAuto*0.8, maxAngVelAuto*0.8);
  }
  
  // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel. 
  // fieldRelative determines field-oriented control vs. robot-oriented control. field-relative control is automatically disabled in the case of a gyro failure.
  // Center of Rotation variables define where the robot will rotate from. 0,0 corresponds to rotations about the center of the robot. +x is towards the front. +y is to the left side.
  public void drive(double _xVel, double _yVel, double _angVel, boolean fieldRelative, double centerOfRotationX, double centerOfRotationY) {
    xVel = _xVel;
    yVel = _yVel;
    angVel = _angVel*180.0/Math.PI;
    moduleStates = fieldRelative
      ? kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(_xVel, _yVel, _angVel, Rotation2d.fromDegrees(getFusedAng())), new Translation2d(centerOfRotationX, centerOfRotationY))
      : kinematics.toSwerveModuleStates(new ChassisSpeeds(_xVel, _yVel, _angVel), new Translation2d(centerOfRotationX, centerOfRotationY));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxVelTeleop); // Makes sure the calculated velocities are attainable. If they are not, all modules velocities are scaled back.
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      modules[moduleIndex].setSMS(moduleStates[moduleIndex]); // Sets the module angles and velocities.
    }
  }

  // Forces the swerve modules into an x-lock pattern to resist movement. Useful for defense or if the robot must remain stationary. 
  public void xLock() {
    xVel = 0.0;
    yVel = 0.0;
    angVel = 0.0;
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      moduleStates[moduleIndex].speedMetersPerSecond = 0.0;
      moduleStates[moduleIndex].angle = (moduleIndex == 0 || moduleIndex == 2) ? Rotation2d.fromDegrees(45.0) : Rotation2d.fromDegrees(-45.0);
      modules[moduleIndex].setSMS(moduleStates[moduleIndex]);
    }
  }

  // Should be called immediately prior to aimDrive() or driveTo(). Resets the PID controllers. Target angle specifies the first angle that will be demanded.
  public void resetDriveController(double targetAngle) {
    xDriveController.reset(getXPos(), xVel);
    yDriveController.reset(getYPos(), yVel);
    angleDriveController.reset(getAngleDistance(getFusedAng(), targetAngle)*Math.PI/180.0, angVel*Math.PI/180.0);
    atDriveGoal = false;
  }

  // Should be called periodically to rotate the robot to the demanded angle in degrees while translating the robot at the specified speed in meter per second.
  public void aimDrive(double _xVel, double _yVel, double targetAngle, boolean fieldRelative) {
    double angleDistance = getAngleDistance(getFusedAng(), targetAngle);
    atDriveGoal = Math.abs(angleDistance) < angTol;
    double _angVel = angleDriveController.calculate(angleDistance*Math.PI/180.0, 0.0);
    if (atDriveGoal) _angVel = 0.0;

    if (Math.abs(_angVel) > Drivetrain.maxAngVelAuto) {
      _angVel = _angVel > 0.0 ? Drivetrain.maxAngVelAuto : -Drivetrain.maxAngVelAuto;
    }
    drive(_xVel, _yVel, _angVel, fieldRelative, 0.0, 0.0);
  }

  // Should be called periodically to move the robot to a specified position and angle. Units are meters and degrees.
  public void driveTo(double targetX, double targetY, double targetAngle) {
    double xVelSetpoint = xDriveController.calculate(getXPos(), targetX);
    double yVelSetpoint = yDriveController.calculate(getYPos(), targetY);
    boolean atXTarget = Math.abs(getXPos() - targetX) < posTol;
    boolean atYTarget = Math.abs(getYPos() - targetY) < posTol;
    
    double angleDistance = getAngleDistance(getFusedAng(), targetAngle);
    double angVelSetpoint = angleDriveController.calculate(angleDistance*Math.PI/180.0, 0.0);
    boolean atAngTarget = Math.abs(angleDistance) < angTol;

    // Checks to see if all 3 targets have been achieved. Sets velocities to 0 to prevent twitchy robot motions at near 0 velocities.
    atDriveGoal = atXTarget && atYTarget && atAngTarget;
    if (atXTarget) xVelSetpoint = 0.0;
    if (atYTarget) yVelSetpoint = 0.0;
    if (atAngTarget) angVelSetpoint = 0.0;

    // Caps the velocities if the PID controllers return values above the specified maximums.
    if (Math.abs(xVelSetpoint) > maxVelAuto) {
      xVelSetpoint = xVelSetpoint > 0.0 ?  maxVelAuto : -maxVelAuto;
    }
    if (Math.abs(yVelSetpoint) > maxVelAuto) {
      yVelSetpoint = yVelSetpoint > 0.0 ? maxVelAuto : -maxVelAuto;
    }
    if (Math.abs(angVelSetpoint) > maxAngVelAuto) {
      angVelSetpoint = angVelSetpoint > 0.0 ? maxAngVelAuto : -maxAngVelAuto;
    }

    drive(xVelSetpoint, yVelSetpoint, angVelSetpoint, true, 0.0, 0.0);
  }

  // Whether the robot has reached the angle specified in the last call to aimDrive() or driveTo(). Should be called after aimDrive() or driveTo() is called within a period.
  public boolean atDriveGoal() {
    return atDriveGoal;
  }

  // Loads the path. All paths should be loaded during robotInit() since this call is computationally expensive. Each path is stored and refered to by the provided index.
  // pathName: The name of the path in Path Planner
  // initialVel: Robot velocity at the begining of the path. Usually 0. Units: meters per second
  // initialAngVel: Robot angular velocity at the begining of the path. Usually 0. Units: degrees per second
  // initialAngle: Robot's angle at the begining of the path. Units: degrees 
  public void loadPath(String pathName, double initialXVel, double initialYVel, double initialAngleVel, double initialAngle) {
    try {
      paths.add(PathPlannerPath.fromPathFile(pathName).generateTrajectory(new ChassisSpeeds(initialXVel, initialYVel, initialAngleVel*Math.PI/180.0), Rotation2d.fromDegrees(initialAngle), RobotConfig.fromGUISettings()));
    } catch (Exception e) {
        DriverStation.reportError("PathPlanner Error: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Should be called once exactly 1 period prior to the start of calls to followPath() each time a new path is followed. pathIndex starts at 0 and incements by 1 for each path loaded into loadPath().
  public void resetPathController(int pathIndex) {
    xPathController.reset();
    yPathController.reset();
    anglePathController.reset();
    pathTimer.restart();
  }
  
  // Tracks the path. Should be called each period. The path controller should be reset if followPath() is not called for a period or more.
  public void followPath(int pathIndex) {
    if (paths.size() > pathIndex) {
      // Samples the trajectory at the current time.
      PathPlannerTrajectoryState currentGoal = paths.get(pathIndex).sample(pathTimer.get());
      pathXPos = currentGoal.pose.getX();
      pathYPos = currentGoal.pose.getY();
      pathAngPos = currentGoal.pose.getRotation().getDegrees();
      double pathXVel = currentGoal.fieldSpeeds.vxMetersPerSecond;
      double pathYVel = currentGoal.fieldSpeeds.vyMetersPerSecond;
      double pathAngVel = currentGoal.fieldSpeeds.omegaRadiansPerSecond;
      double xVelCorrection = xPathController.calculate(getXPos(), pathXPos);
      double yVelCorrection = yPathController.calculate(getYPos(), pathYPos);
      double angleDistance = getAngleDistance(getFusedAng(), pathAngPos);
      double angVelCorrection = anglePathController.calculate(angleDistance*Math.PI/180.0, 0.0);
      double angVelSetpoint = pathAngVel + angVelCorrection;
      double xVelSetpoint = pathXVel + xVelCorrection;
      double yVelSetpoint = pathYVel + yVelCorrection;

      // Checks to see if all 3 targets have been achieved. Sets velocities to 0 to prevent twitchy robot motions at near 0 velocities.
      atDriveGoal = atPathEndpoint(pathIndex);
      if (atDriveGoal) {
        xVelSetpoint = 0.0;
        yVelSetpoint = 0.0;
        angVelSetpoint = 0.0;
      }

      // Caps the velocities if the PID controllers return values above the specified maximums.
      if (Math.abs(xVelSetpoint) > maxVelAuto) {
        xVelSetpoint = xVelSetpoint > 0.0 ?  maxVelAuto : -maxVelAuto;
      }
      if (Math.abs(yVelSetpoint) > maxVelAuto) {
        yVelSetpoint = yVelSetpoint > 0.0 ? maxVelAuto : -maxVelAuto;
      }
      if (Math.abs(angVelSetpoint) > maxAngVelAuto) {
        angVelSetpoint = angVelSetpoint > 0.0 ? maxAngVelAuto : -maxAngVelAuto;
      }

      drive(xVelSetpoint, yVelSetpoint, angVelSetpoint, true, 0.0, 0.0);
    } else {
      drive(0.0, 0.0, 0.0, false, 0.0, 0.0);
    }
  }
  
  // Tells whether the robot has reached the endpoint of the path, within the specified tolerance.
  // pathIndex: Which path to check, pathXTol and pathYTol: the allowable difference in position in meters, pathAngTol: the allowable difference in angle in degrees
  public boolean atPathEndpoint(int pathIndex) {
    if (paths.size() > pathIndex) {
    PathPlannerTrajectoryState endState = paths.get(pathIndex).getEndState();
    return Math.abs(getFusedAng() - endState.pose.getRotation().getDegrees()) < angTol 
      && Math.abs(getXPos() - endState.pose.getX()) < posTol 
      && Math.abs(getYPos() - endState.pose.getY()) < posTol;
    } else {
      return false;
    }
  }

  // Updates the position of the robot on the field. Should be called each period to remain accurate. Tends to noticably drift for periods of time >15 sec.
  public void updateOdometry() {
    BaseStatusSignal.waitForAll(0.008, pigeonYaw, pigeonYawRate, pigeonPitch, pigeonPitchRate, pigeonRoll, pigeonRollRate,
    frontLeftModule.driveMotorPosition, frontLeftModule.driveMotorVelocity, frontLeftModule.wheelEncoderPosition, frontLeftModule.wheelEncoderVelocity,
    frontRightModule.driveMotorPosition, frontRightModule.driveMotorVelocity, frontRightModule.wheelEncoderPosition, frontRightModule.wheelEncoderVelocity,
    backRightModule.driveMotorPosition, backRightModule.driveMotorVelocity, backRightModule.wheelEncoderPosition, backRightModule.wheelEncoderVelocity,
    backLeftModule.driveMotorPosition, backLeftModule.driveMotorVelocity, backLeftModule.wheelEncoderPosition, backLeftModule.wheelEncoderVelocity);
    
    odometry.update(Rotation2d.fromDegrees(getGyroAng()), getModulePositions());
  }

  // Communicates the robot's heading to the Limelight. Should be called each period, and before any calls to addVisionEstimate() or addCalibrationEstimate()
  public void updateVisionHeading(boolean knownHeading, double heading) {
    for (int limelightIndex = 0; limelightIndex < limelights.length; limelightIndex++) { // Iterates through each limelight.
      double blueHeading;
      if (knownHeading) {
        blueHeading = isBlueAlliance() ? heading : heading - 180.0; // Converts the robot's angular position to the blue coordinate system.
      } else {
        blueHeading = isBlueAlliance() ? getFusedAng() : getFusedAng() - 180.0; // Converts the robot's angular position to the blue coordinate system.
      }
      LimelightHelpers.SetRobotOrientation(limelights[limelightIndex], blueHeading, getGyroAngVel(), getGyroPitch(), getGyroPitchVel(), getGyroRoll(), getGyroRollVel()); // Communicates the robot's heading to the Limelight.
    }
  }

  // Determines which limelight will provide the most accurate localization data. Returns -1 if no limelights are initialized.
  public void calcPriorityLimelightIndex() {
    if (limelights.length > 0) {
      // Determines the amount of area covered by April Tags in each of the cameras.
      double[] tagArea = new double[limelights.length];
      for (int index = 0; index < limelights.length; index++) {
        PoseEstimate botpose = isBlueAlliance() ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelights[index]) : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelights[index]); 
        if (botpose != null) {
          tagArea[index] = botpose.avgTagArea*botpose.tagCount;          
        } else {
          tagArea[index] = 0.0;
        }
      }

      // Determines which camera has the largest amount of area covered by April Tags.
      double maxTagArea = tagArea[0];
      int maxTagAreaIndex = 0;
      for (int index = 1; index < limelights.length; index++) {
        if (tagArea[index] > maxTagArea) {
          maxTagArea = tagArea[index];
          maxTagAreaIndex = index;
        }
      }
      priorityLimelightIndex = maxTagAreaIndex;
    } else {
      priorityLimelightIndex = -1;
    }
  }

  // Returns the index of the limelight that will provide the most accurate localization data. calcPriorityLimelightIndex() should be called in advance to update this value.
  public int getPriorityLimelightIndex() {
    return priorityLimelightIndex;
  }
  
  // Incorporates vision information to determine the position of the robot on the field. Should be used only when vision information is deemed to be highly reliable (>1 april tag, close to april tag...)
  // limelightIndex indicates the camera to use. 0 is corresponds to the first entry in the limelights[] array. 
  public void addVisionEstimate(int limelightIndex, boolean megaTag2) {
    if (limelights.length > limelightIndex) {
      PoseEstimate botpose;
      if (megaTag2) {
        botpose = isBlueAlliance() ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelights[limelightIndex]) : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelights[limelightIndex]); // Transforms the vision position estimate to the appropriate coordinate system for the robot's alliance color
      } else {
        botpose = isBlueAlliance() ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelights[limelightIndex]) : LimelightHelpers.getBotPoseEstimate_wpiRed(limelights[limelightIndex]);
      }
      if (botpose != null) {
        double SD = 0.5; // How much variance there is in the LL vision information. Lower numbers indicate more trustworthy data.
        if (botpose.tagCount >= 1) { // At least 1 AprilTag is detected.
          if (botpose.avgTagArea*botpose.tagCount > 2.0 && Math.sqrt(Math.pow(getXVel(), 2) + Math.pow(getYVel(), 2)) < 0.5 && Math.abs(getAngVel()) < 45.0 && Math.abs(getGyroPitch()) < 2.0 && Math.abs(getGyroRoll()) < 2.0) { // The robot is relatively stationary, flat, and the AprilTag is very close to the robot.
            SD = 0.1; // Reduces the standard deviation of the vision estimate.
            accurateCalibrationTimer.restart();
          }
          odometry.setVisionMeasurementStdDevs(VecBuilder.fill(SD, SD, Double.MAX_VALUE));
          odometry.addVisionMeasurement(new Pose2d(botpose.pose.getX(), botpose.pose.getY(), Rotation2d.fromDegrees(getFusedAng())), botpose.timestampSeconds);  
          calibrationTimer.restart();
        }
      } 
    }
  }

  // Should be called during disabledInit(). Wipes previous calibration data from the calibrator.
  public void resetCalibration() {
    calibrationArray = new double[3][maxCalibrationFrames];
    calibrationIndex = 0;
    calibrationFrames = 0;
  }

  // Should be called during disabled(). Calibrates the robot's starting position based on any April Tags in sight of the Limelight.
  // limelightIndex indicates the camera to use. 0 is corresponds to the first entry in the limelights[] array. 
  public void addCalibrationEstimate(int limelightIndex, boolean megaTag2) {
    if (limelights.length > limelightIndex) {
      PoseEstimate botpose;
      if (megaTag2) {
        botpose = isBlueAlliance() ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelights[limelightIndex]) : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelights[limelightIndex]); // Transforms the vision position estimate to the appropriate coordinate system for the robot's alliance color
      } else {
        botpose = isBlueAlliance() ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelights[limelightIndex]) : LimelightHelpers.getBotPoseEstimate_wpiRed(limelights[limelightIndex]);
      }
      if (botpose != null) {
        if (botpose.tagCount > 0) { // Checks to see whether there is at least 1 vision target and the LL has provided a new frame.
          calibrationArray[0][calibrationIndex] = botpose.pose.getX(); // Adds an x-position entry to the calibrationPosition array. 
          calibrationArray[1][calibrationIndex] = botpose.pose.getY(); // Adds a y-position entry to the calibrationPosition array. 
          calibrationArray[2][calibrationIndex] = botpose.pose.getRotation().getDegrees(); // Adds a angle-position entry to the calibrationPosition array. 
          calibrationIndex = (calibrationIndex + 1) % maxCalibrationFrames; // Handles the looping of the calibrationIndex variable. 
          if (calibrationFrames < maxCalibrationFrames) calibrationFrames++;  // Increments calibrationPoints until the calibrationPosition array is full.
          calibrationTimer.restart();
        } 
      }
    }
  }

  // Should be called during autoInit() or teleopInit() to update the robot's starting position based on its April Tag calibration
  public void pushCalibration(boolean knownHeading, double heading) {
    if (calibrationFrames > minCalibrationFrames) {
      double[] calibrationSum = new double[5];
      for (int calibrationIndex = 0; calibrationIndex < calibrationFrames; calibrationIndex++) {
        calibrationSum[0] = calibrationSum[0] + calibrationArray[0][calibrationIndex];
        calibrationSum[1] = calibrationSum[1] + calibrationArray[1][calibrationIndex];
        calibrationSum[2] = calibrationSum[2] + Math.sin(calibrationArray[2][calibrationIndex]*Math.PI/180.0);
        calibrationSum[3] = calibrationSum[3] + Math.cos(calibrationArray[2][calibrationIndex]*Math.PI/180.0);
        calibrationSum[4] = calibrationSum[4] + Math.abs(calibrationArray[2][calibrationIndex]);
      }
      double calibrationAng;
      if (knownHeading) {
        calibrationAng = Math.toRadians(heading);
      } else {
        calibrationAng = calibrationSum[4]/calibrationFrames > 90.0 ? Math.atan(calibrationSum[2]/calibrationSum[3]) + Math.PI : Math.atan(calibrationSum[2]/calibrationSum[3]);
      }
      odometry.resetPosition(Rotation2d.fromDegrees(getGyroAng()), getModulePositions(), new Pose2d(calibrationSum[0]/calibrationFrames, calibrationSum[1]/calibrationFrames, Rotation2d.fromRadians(calibrationAng))); // Averages the values in the calibrationPosition Array and sets the robot position based on the averages.
      calibrationTimer.restart();
    }
  }

  // Returns the amount of time that has elapsed since the robot has updated its position on the field using vision.
  public double getCalibrationTimer() {
    return calibrationTimer.get();
  }

  // Returns the amount of time that has elapsed since the robot has updated its position on the field using a very trustworthy vision estimate.
  public double getAccurateCalibrationTimer() {
    return accurateCalibrationTimer.get();
  }

  // Resets the gyro to 0 based on the current orientation of the robot.
  public void resetGyro() {
    pigeon.setYaw(0.0);
    odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d(getXPos(), getYPos(), new Rotation2d()));
  }
  
  // Returns the angular position of the robot in degrees. The angular position is referenced to the starting angle of the robot. CCW is positive. Will return 0 in the case of a gyro failure.
  public double getGyroAng() {
    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(pigeonYaw, pigeonYawRate, 0.02);
  }

  // Returns the angular velocity of the robot in degrees per second. CCW is positive.
  public double getGyroAngVel() {
    return pigeonYawRate.getValueAsDouble();
  }

  // Returns the pitch of the robot in degrees. An elevated front is positive. An elevated rear is negative.
  public double getGyroPitch() {
    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(pigeonPitch, pigeonPitchRate, 0.02);
  }
  
  // Returns the pitch velocity of the robot in degrees per second.
  public double getGyroPitchVel() {
    return pigeonPitchRate.getValueAsDouble();
  }

  // Returns the roll of the robot in degrees. An elevated left side is positive. An elevated right side is negative.
  public double getGyroRoll() {
    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(pigeonRoll, pigeonRollRate, 0.02);
  }

  // Returns the roll velocity of the robot in degrees per second.
  public double getGyroRollVel() {
    return pigeonRollRate.getValueAsDouble();
  }

  // Returns true if the robot is on the red alliance.
  public boolean isRedAlliance() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get().equals(Alliance.Red);
    } else {
      return false;
    }
  }

  // Returns true if the robot is on the blue alliance.
  public boolean isBlueAlliance() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get().equals(Alliance.Blue);
    } else {
      return false;
    }
  }

  // Returns the last commanded x-velocity of the robot in meters per second.
  public double getXVel() {
    return xVel;
  }

  // Returns the last commanded y-velocity of the robot in meters per second.
  public double getYVel() {
    return yVel;
  }

  // Returns the last commanded angular-velocity of the robot in degrees per second.
  public double getAngVel() {
    return angVel;
  }
  
  // Returns the odometry calculated x position of the robot in meters. This is based on vision and gyro data combined.
  public double getXPos() {
    return odometry.getEstimatedPosition().getX();
  }

  // Returns the odometry calculated y position of the robot in meters. This is based on vision and gyro data combined.
  public double getYPos() {
    return odometry.getEstimatedPosition().getY();
  }

  // Returns the odometry calcualted angle of the robot in degrees. This is based on vision and gyro data combined.
  public double getFusedAng() {
    return odometry.getEstimatedPosition().getRotation().getDegrees();
  }
  
  // The distance between the robot's current position and the current trajectory position. Units: meters
  public double getPathPosError() {
    return Math.sqrt(Math.pow(pathYPos - getYPos(), 2) + Math.pow(pathXPos - getXPos(), 2));
  }

  // The angular distance to the current trajectory point. Units: degrees
  public double getPathAngleError() {
    return getAngleDistance(getFusedAng(), pathAngPos);
  }
  
  // Publishes information to the dashboard. Should be called each period.
  public void updateDash() {
    //SmartDashboard.putNumber("Vision Calibration Timer", getCalibrationTimer());
    //SmartDashboard.putBoolean("atDriveGoal", atDriveGoal);
    //SmartDashboard.putNumber("Front Left Swerve Module Position", frontLeftModule.getDriveMotorPos());
    //SmartDashboard.putNumber("Front Right Swerve Module Position", frontRightModule.getDriveMotorPos());
    //SmartDashboard.putNumber("Back Right Swerve Module Position", backRightModule.getDriveMotorPos());
    //SmartDashboard.putNumber("Back LeftF Swerve Module Position", backLeftModule.getDriveMotorPos());
    //SmartDashboard.putNumber("Front Left Swerve Module Wheel Encoder Angle", frontLeftModule.getWheelAngle());
    //SmartDashboard.putNumber("Front Right Swerve Module Wheel Encoder Angle", frontRightModule.getWheelAngle());
    //SmartDashboard.putNumber("Back Right Swerve Module Wheel Encoder Angle", backRightModule.getWheelAngle());
    //SmartDashboard.putNumber("Back Left Swerve Module Wheel Encoder Angle", backLeftModule.getWheelAngle());
    //SmartDashboard.putNumber("Robot X Position", getXPos());
    //SmartDashboard.putNumber("Robot Y Position", getYPos());
    //SmartDashboard.putNumber("Robot Angular Position (Fused)", getFusedAng());
    //SmartDashboard.putNumber("Robot Angular Position (Gyro)", getGyroAng());
    //SmartDashboard.putNumber("Robot Pitch", getGyroPitch());
    //SmartDashboard.putNumber("Robot Roll", getGyroRoll());
    //SmartDashboard.putNumber("Robot Angular Rate", getGyroAngVel());
    //SmartDashboard.putNumber("Robot Pitch Rate", getGyroPitchVel());
    //SmartDashboard.putNumber("Robot Roll Rate", getGyroRollVel());
    //SmartDashboard.putNumber("Robot Demanded X Velocity", getXVel());
    //SmartDashboard.putNumber("Robot Demanded Y Velocity", getYVel());
    //SmartDashboard.putNumber("Robot Demanded Angular Velocity", getAngVel());
    //SmartDashboard.putNumber("Path X Position", pathXPos);
    //SmartDashboard.putNumber("Path Y Position", pathYPos);
    //SmartDashboard.putNumber("Path Angular Position", pathAngPos);
    //SmartDashboard.putNumber("Path Position Error", getPathPosError());
    //SmartDashboard.putNumber("Path Angle Error", getPathAngleError());
    //SmartDashboard.putBoolean("Path At Endpoint", atPathEndpoint(0));
    //SmartDashboard.putBoolean("isRedAllaince", isRedAlliance());
    //SmartDashboard.putBoolean("isBlueAllaince", isBlueAlliance());   
  }

  // Calculates the shortest distance between two points on a 360 degree circle. CW is + and CCW is -
  public double getAngleDistance(double currAngle, double targetAngle) {
    double directDistance = Math.abs(currAngle - targetAngle);
    double wraparoundDistance = 360.0 - directDistance;
    double minimumDistance = Math.min(directDistance, wraparoundDistance);
    boolean isCW = (currAngle > targetAngle && wraparoundDistance > directDistance) || (currAngle < targetAngle && wraparoundDistance < directDistance);
    if (!isCW) {
      minimumDistance = -minimumDistance;
    }
    return minimumDistance;
  }

  // Returns an array that contains each swerve module's drive motor position and swerve wheel position.
  private SwerveModulePosition[] getModulePositions() {
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      modulePositions[moduleIndex] = modules[moduleIndex].getSMP();
    }
    return modulePositions;
  }
}