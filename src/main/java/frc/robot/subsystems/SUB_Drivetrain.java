// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Swerve;
import frc.robot.utils.*;

/**
 * The SUB_Drivetrain class manages the robot's swerve drive system.
 * It encapsulates the four swerve modules, gyro, kinematics, and odometry.
 * It provides methods for controlling the robot's movement, including field-relative driving,
 * and for querying the robot's state, such as its pose and velocity.
 * This class follows a singleton pattern to ensure only one instance is created.
 */

// Notice the extends keyword, this means that SUB_Drivetrain inherits from SubsystemBase
// SubsystemBase is a class provided by wpilib and allows us to do things like requiring the subsystem and to other useful commands
public class SUB_Drivetrain extends SubsystemBase {

  /** A publisher for sending odometry data to AdvantageScope for visualization. */
  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("AdvantageScopeOdometry", Pose2d.struct).publish();

  /** A publisher for debugging a target X point on the field. */
  public StructPublisher<Pose2d> publisher1 = NetworkTableInstance.getDefault()
  .getStructTopic("debugXPoint", Pose2d.struct).publish(); 

  /** A publisher for debugging a target Y point on the field. */
  public StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault()
  .getStructTopic("debugYPoint", Pose2d.struct).publish(); 

  /** A publisher for the pose estimated by PhotonVision camera 1. */
  public StructPublisher<Pose2d> publisher3 = NetworkTableInstance.getDefault()
  .getStructTopic("PhotonCam1Pose", Pose2d.struct).publish(); 

  /** A publisher for the pose estimated by PhotonVision camera 2. */
  public StructPublisher<Pose2d> publisher4 = NetworkTableInstance.getDefault()
  .getStructTopic("PhotonCam2Pose", Pose2d.struct).publish(); 

  /** A publisher for the final selected pose after sensor fusion. */
  public StructPublisher<Pose2d> selectPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("SelectedPose", Pose2d.struct).publish(); 

  /** A publisher for the current states of the swerve modules. */
  private StructArrayPublisher<SwerveModuleState> currentStatePublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Current States", SwerveModuleState.struct).publish();

  /** A publisher for the desired states of the swerve modules. */
  private StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();

  /** The field visualization object for displaying the robot's pose on SmartDashboard. */
  public final Field2d m_field = new Field2d();

  /** The singleton instance of the drivetrain subsystem. */
  // Sets up for the getInstance() method below, this has to be static so that it is shared across all instances of the class or else none of this would work.
  private static SUB_Drivetrain INSTANCE = null;

  // Declares the motors and other components of the drivetrain   
  // Notice the use of the new keyword to call constructors of classes provided by wpilib and our own custom classes.
  // Also notice the use of Constants.XXXX. This is how we access the values defined in Constants.java
  // Notice that in java we can use multi line statements because of the use of semicolons to end statements.
  /** The front-left swerve module instance. */
  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(Constants.Drivetrain.kFRONT_LEFT_DRIVE_MOTOR_CANID,
          Constants.Drivetrain.kFRONT_LEFT_STEER_MOTOR_CANID,
          Constants.Drivetrain.kFrontLeftChassisAngularOffset);

  /** The front-right swerve module instance. */
  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(Constants.Drivetrain.kFRONT_RIGHT_DRIVE_MOTOR_CANID,
          Constants.Drivetrain.kFRONT_RIGHT_STEER_MOTOR_CANID,
          Constants.Drivetrain.kFrontRightChassisAngularOffset);

  /** The back-left swerve module instance. */
  private final MAXSwerveModule backLeft =
      new MAXSwerveModule(Constants.Drivetrain.kBACK_LEFT_DRIVE_MOTOR_CANID,
          Constants.Drivetrain.kBACK_LEFT_STEER_MOTOR_CANID,
          Constants.Drivetrain.kBackLeftChassisAngularOffset);

  /** The back-right swerve module instance. */
  private final MAXSwerveModule backRight =
      new MAXSwerveModule(Constants.Drivetrain.kBACK_RIGHT_DRIVE_MOTOR_CANID,
          Constants.Drivetrain.kBACK_RIGHT_STEER_MOTOR_CANID,
          Constants.Drivetrain.kBackRightChassisAngularOffset);

  /** An array containing all the swerve modules for easy iteration. */
  private MAXSwerveModule[] modules =
      new MAXSwerveModule[] {frontLeft, frontRight, backLeft, backRight};

  /** An array to hold the current states of the swerve modules. */
  private SwerveModuleState[] moduleStates = getModuleStates();

  /** The desired chassis speeds, used for velocity control. */
  private ChassisSpeeds setpoint = new ChassisSpeeds();

  /** The layout of the AprilTags on the field, used for pose estimation. */
  public AprilTagFieldLayout at_field;

  /** The calculated field-relative velocity of the robot. */
  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();

  /** The last known field-relative velocity, used for calculating acceleration. */
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();

  /** The calculated field-relative acceleration of the robot. */
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();;

  /** The NavX gyro for measuring robot heading and rotation. */
  private AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  /** The current rotational velocity of the robot. */
  private double m_currentRotation = 0.0;

  /** The current direction of translation of the robot. */
  private double m_currentTranslationDir = 0.0;

  /** The current magnitude of translation of the robot. */
  private double m_currentTranslationMag = 0.0;

  /** A slew rate limiter for the magnitude of translation to smooth joystick inputs. */
  private SlewRateLimiter m_magLimiter =
      new SlewRateLimiter(Constants.Drivetrain.kMagnitudeSlewRate);

  /** A slew rate limiter for the rotation of the robot to smooth joystick inputs. */
  private SlewRateLimiter m_rotLimiter =
      new SlewRateLimiter(Constants.Drivetrain.kRotationalSlewRate);

  /** The previous timestamp, used for calculating elapsed time in rate limiting. */
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  /** The current pose of the robot on the field. */
  private Pose2d pose = new Pose2d();

  /** The pose estimator for fusing sensor data to determine robot pose. */
  public SwerveDrivePoseEstimator m_poseEstimator;

  /**
   * Returns the singleton instance of the drivetrain subsystem.
   * Ensures that only one instance of the drivetrain is created.
   *
   * @return The singleton instance of the SUB_Drivetrain.
   */
  // This is a public static method to get the instance of the drivetrain subsystem
  // Notice how this is static, this means that it is shared across all instances of the class so that we can make sure there is only one instance
  // Also notice how it has a return type of SUB_Drivetrain and is public so other classes can call it
  public static SUB_Drivetrain getInstance() {
    // If there is no instance yet, create one by using the constructor below
    // This constructor is private so that no other class can call it and create another instance
    if (INSTANCE == null) {
      INSTANCE = new SUB_Drivetrain();
    }

    //If there is already an instance, just return it
    return INSTANCE;
  }

  /**
   * Constructs a new SUB_Drivetrain.
   * It initializes the pose estimator and zeroes the gyro heading.
   */
  private SUB_Drivetrain() {
    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.Drivetrain.kDriveKinematics,
    Rotation2d.fromDegrees(getAngle()),
    new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
        backLeft.getPosition(), backRight.getPosition()},
    new Pose2d(0, 0, new Rotation2d(0)));
    zeroHeading();
    
  }

  /**
   * This method is called periodically every robot loop (approximately every 20ms).
   * It updates the pose estimator, calculates robot speed and acceleration,
   * and sends telemetry data to SmartDashboard and AdvantageScope.
   * This allows the drivers to see the robot's position on the field and other useful information.
   */
  @Override
  // Notice how this function has a void return type, this means that it does not return any value so it cant be used like so: int x = periodic();.
  public void periodic() {

    m_poseEstimator.update(Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()});
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    modules = new MAXSwerveModule[] {frontLeft, frontRight, backLeft, backRight};

    m_field.setRobotPose(getPose());

    m_fieldRelVel = new FieldRelativeSpeed(
        Constants.Drivetrain.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
            frontRight.getState(), backLeft.getState(), backRight.getState()),
        navx.getRotation2d());
    m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, 0.02);
    m_lastFieldRelVel = m_fieldRelVel;

    publisher.set(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putNumberArray("Drive/PoseEstimator",
        new double[] {m_poseEstimator.getEstimatedPosition().getX(),
            m_poseEstimator.getEstimatedPosition().getY(),
            m_poseEstimator.getEstimatedPosition().getRotation().getDegrees()});

    SmartDashboard.putData("Drive/Field", m_field);
    SmartDashboard.putNumberArray("Odometry",
        new double[] {getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()});

    SmartDashboard.putNumber("Drive/Robot Pose X meters", (getPose().getX()));
    SmartDashboard.putNumber("Drive/Robot Pose Y meters", (getPose().getY()));
    SmartDashboard.putNumber("Drive/rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Drive/Robot Speed", modules[0].getVelocityDrive());

    SmartDashboard.putNumber("BACK RIGHT MODULE POSITION", backRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("BACK LEFT MODULE POSITION", backLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("FRONT LEFT MODULE POSITION", frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("FRONT RIGHT MODULE POSITION",
        frontRight.getPosition().distanceMeters);



    currentStatePublisher.set(getModuleStates());

    SmartDashboard.putNumber("NavX angle", Units.degreesToRadians(getAngle()));

  }

  // Regular methods start here
  // For new coders: You might see a lot of methods that seem very useless that just return one value
  // However, they are neccecary because motors are defined as private meaning they shouldn't be accessed from other classes and these public methods allow other classes to interface with the motors

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The robot's pose as a Pose2d object.
   */
  public Pose2d getPose() {
    Pose2d pose =  m_poseEstimator.getEstimatedPosition(); // Why not just return m_poseEstimator.getEstimatedPosition()?
    return pose;
  }

  /**
   * Resets the robot's odometry to a specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()},
        pose);

    this.pose = pose;
  }

  // The below comments are called doxygen comments and are used to give those useful popups when hovering over a method.
  // They define a description of the method, its parameters, and its return value using @breif @param and @return respectivley.
  // They also are used to generate documentation automatically
  // This is great to know what a method does and you will see it used in many wpilib methods and classes.
  /**
   * Drives the robot using joystick inputs.
   *
   * @param xSpeed The speed of the robot in the x direction (forward/backward).
   * @param ySpeed The speed of the robot in the y direction (strafe left/right).
   * @param rot The angular rate of the robot (rotation).
   * @param fieldRelative True to drive relative to the field, false to drive relative to the robot.
   * @param rateLimit True to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
      boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate =
            Math.abs(Constants.Drivetrain.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // a high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
            inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // a small number to avoid floating-point errors
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
            inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    // Notice the use of * for multiplication
    double xSpeedDelivered = xSpeedCommanded * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Constants.Drivetrain.kMaxAngularSpeed;

    // Convert chassis speeds to individual module states, handling field-relative translation
    // Notice the use of the var keyword, this allows the compiler to infer the type of the variable based on what it is being set to
    // This is useful to reduce redundancy and make the code cleaner with complex types but does not mean that the type can change during runtime
    // This is unlike languages like python where types can be changed at runtime instead of being determined at compile time
    // Dont use the var keyword for everything though as it can make it hard to determine the type of a variable
    // var also cant be used for the type of a function parameter or return type
    // The var keyword can not be declared like so: var x; new line x = 5; because x = 5; could be in an if statement and it could get set to a different type meaning it wouldnt be determined at compile time makingit invalid.
    // This means the var keyword has to be used like so var x = 5;
    var swerveModuleStates =
        Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        Constants.Drivetrain.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    desiredStatePublisher.set(swerveModuleStates);
  }

  /** Commands the swerve modules to form an "X" shape, preventing movement. */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Commands all swerve modules to a specific angle.
   * @param angle The angle in degrees to set the modules to.
   */
  public void setAngle(double angle) {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
  }

  /**
   * Sets the desired states for each swerve module.
   *
   * @param desiredStates An array of the desired SwerveModuleState for each module.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        Constants.Drivetrain.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders on all swerve modules to zero. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot by resetting the NavX gyro. */
  public void zeroHeading() {
    navx.zeroYaw();
    m_poseEstimator.resetRotation(Rotation2d.fromDegrees(navx.getAngle()));
  }


  /**
   * Returns the raw angle of the robot from the NavX gyro.
   * @return The angle of the robot in degrees.
   */
  public double getAngle() {
    return -navx.getAngle();
  }

  /**
   * Returns the heading of the robot in degrees, from -180 to 180.
   *
   * @return The robot's heading.
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  /**
   * Returns the rotation of the robot as a Rotation2d object.
   * @return The rotation of the robot.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  /**
   * Returns the turn rate of the robot from the NavX gyro.
   *
   * @return The turn rate in degrees per second.
   */
  public double getTurnRate() {
    return navx.getRate() * (Constants.Drivetrain.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the current states of the swerve modules.
   * @return An array of SwerveModuleState objects.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    // Notice the use of for loops to iterate through arrays
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }


  /**
   * Returns the current positions of the swerve modules.
   * @return An array of SwerveModulePosition objects.
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

    for (int i = 0; i < moduleStates.length; i++) {
      // Notice the use of for loops to iterate through arrays
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Returns the current chassis speeds of the robot.
   * @return The chassis speeds as a ChassisSpeeds object.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return Drivetrain.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Resets the robot's pose in the pose estimator.
   * @param pose The new pose of the robot.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getRotation2d(), getPositions(), pose);

    this.pose = pose;
  }

  /**
   * Drives the robot with field-relative speeds.
   * @param fieldRelativeSpeeds The desired field-relative speeds.
   */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  /**
   * Drives the robot with robot-relative speeds.
   * @param robotRelativeSpeeds The desired robot-relative speeds.
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

    ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
      robotRelativeSpeeds.vxMetersPerSecond,
      robotRelativeSpeeds.vyMetersPerSecond,
      robotRelativeSpeeds.omegaRadiansPerSecond //Unstable.
    );
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates =
        Drivetrain.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Sets the desired translational velocity of the robot.
   * @param speeds The desired chassis speeds (vx and vy).
   */
  public void driveVelocity(ChassisSpeeds speeds) {
    setpoint.vxMetersPerSecond = speeds.vxMetersPerSecond;
    setpoint.vyMetersPerSecond = speeds.vyMetersPerSecond;
  }

  /**
   * Sets the desired rotational velocity of the robot.
   * @param omega The desired rotational velocity in radians per second.
   */
  public void driveVelocity(double omega) {
    setpoint.omegaRadiansPerSecond = omega;
  }

  /** Stops the robot's movement completely. */
  public void stop() {
    driveVelocity(new ChassisSpeeds());
  }

  /**
   * Returns a command that drives the robot while maintaining a PID-controlled heading.
   * @param headingSupplier A supplier for the desired heading as an Optional<Rotation2d>.
   * @return A command that controls the robot's heading.
   */
  public Command pidControlledHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
    var subsystem = this;
    return new Command() {
        private final PIDController headingPID = new PIDController(Swerve.kDrivingP, Swerve.kDrivingI, Swerve.kDrivingD);
        {
            addRequirements(subsystem);
            setName("PID Controlled Heading");
            headingPID.enableContinuousInput(-Math.PI, Math.PI); // Enable continuous input
            headingPID.setTolerance(Swerve.headingTolerance);
        }
        private Rotation2d desiredHeading;
        private boolean headingSet;

        @Override
        public void initialize() {
            desiredHeading = getPose().getRotation();
        }

        @Override
        public void execute() {
            var heading = headingSupplier.get();
            headingSet = heading.isPresent();
            heading.ifPresent((r) -> desiredHeading = r);
            double turnInput = headingPID.calculate(getPose().getRotation().getRadians(), desiredHeading.getRadians());
            turnInput = headingPID.atSetpoint() ? 0 : turnInput;
            turnInput = MathUtil.clamp(turnInput, -0.5, +0.5);
            driveVelocity(turnInput * Swerve.kMaxRotationalSpeed);
        }

        @Override
        public void end(boolean interrupted) {
            stop();
        }

        @Override
        public boolean isFinished() {
            return !headingSet && headingPID.atSetpoint();
        }
    };
}

  /**
   * Returns a command that drives the robot with field-relative speeds.
   * @param speeds A supplier for the desired field-relative chassis speeds.
   * @return A command that drives the robot field-relatively.
   */
  public Command fieldRelative(Supplier<ChassisSpeeds> speeds) {
    var subsystem = this;
    return new Command() {
      {
        addRequirements(subsystem);
        setName("Field Relative");
      }

      @Override
      public void execute() {
        driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getPose().getRotation()));
      }

      @Override
      public void end(boolean interrupted) {
        stop();
      }
    };
  }

  /**
   * Returns a command that points the robot towards a specific position on the field.
   * @param posToPointTo A supplier for the target position as an Optional<Translation2d>.
   * @param forward A supplier for the desired forward direction of the robot.
   * @return A command that points the robot to the specified position.
   */
  public Command pointTo(Supplier<Optional<Translation2d>> posToPointTo,
      Supplier<Rotation2d> forward) {
    return pidControlledHeading(() -> posToPointTo.get().map((pointTo) -> {
      var FORR = pointTo.minus(getPose().getTranslation());
      return new Rotation2d(FORR.getX(), FORR.getY()).minus(forward.get());
    }));
  }

  /**
   * Adds a vision-based pose measurement to the pose estimator.
   * This is used to correct the robot's estimated pose with data from cameras.
   * 
   * @param visionPose The pose estimated by the vision system.
   * @param timestampSeconds The timestamp of the vision measurement.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  /**
   * Returns the current field-relative speed of the robot.
   * @return The field-relative speed as a FieldRelativeSpeed object.
   */
  public FieldRelativeSpeed getFieldRelativeSpeed() {
    return m_fieldRelVel;
  }

  /**
   * Returns the current field-relative acceleration of the robot.
   * @return The field-relative acceleration as a FieldRelativeAccel object.
   */
  public FieldRelativeAccel getFieldRelativeAccel() {
    return m_fieldRelAccel;
  }

}