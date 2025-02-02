package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import com.pathplanner.lib.auto.AutoBuilder;

import drivers.PhoenixSwerveHelper;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import lombok.Getter;
import util.AllianceFlipUtil;
import util.RunnableUtil.RunOnce;
import util.ScreamUtil;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends frc.robot.subsystems.drivetrain.generated.TunerConstants.TunerSwerveDrivetrain implements Subsystem {
  private double lastSimTime;

    public final SwerveDrivePoseEstimator m_PoseEstimate = new SwerveDrivePoseEstimator(getKinematics(), getPigeon2().getRotation2d(), getState().ModulePositions, new Pose2d());
  private RunOnce operatorPerspectiveApplier = new RunOnce();

  @Getter private final PhoenixSwerveHelper helper;

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    CommandScheduler.getInstance().registerSubsystem(this);

    helper =
        new PhoenixSwerveHelper(
            this::getPose,
            DrivetrainConstants.MAX_SPEED,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS);

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        () -> getState().Speeds,
        (speeds, feedforwards) ->
            setControl(
                helper
                    .getApplyRobotSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        DrivetrainConstants.PATH_FOLLOWING_CONTROLLER,
        DrivetrainConstants.ROBOT_CONFIG,
        () -> false,
        this);

    System.out.println("[Init] Drivetrain initialization complete!");
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /* public Command driveToBargeScoringZone(Supplier<Translation2d> translation) {
    return applyRequest(
            () ->
                helper.getFacingAngle(
                    new Translation2d(
                        DrivetrainConstants.X_ALIGNMENT_CONTROLLER.calculate(
                                getPose().getX(),
                                AllianceFlipUtil.get(
                                    FieldConstants.BLUE_BARGE_ALIGN_X,
                                    FieldConstants.RED_BARGE_ALIGN_X))
                            * AllianceFlipUtil.getDirectionCoefficient(),
                        translation.get().getY()),
                    AllianceFlipUtil.getFwdHeading()))
        .beforeStarting(
            () ->
                DrivetrainConstants.X_ALIGNMENT_CONTROLLER.reset(
                    getPose().getX(),
                    getLinearVelocity().getNorm() * AllianceFlipUtil.getDirectionCoefficient()));
  } */



  public void updateSimState() {
    final double currentTime = Utils.getCurrentTimeSeconds();
    double deltaTime = currentTime - lastSimTime;
    lastSimTime = currentTime;

    updateSimState(deltaTime, RobotController.getBatteryVoltage());
  }

  public boolean getWithinAngleThreshold(Rotation2d targetAngle, Rotation2d threshold) {
    return ScreamUtil.withinAngleThreshold(targetAngle, getHeading(), threshold);
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[DrivetrainConstants.NUM_MODULES];
    for (int i = 0; i < DrivetrainConstants.NUM_MODULES; i++) {
      states[i] = getModules()[i].getCurrentState();
    }
    return states;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  public Translation2d getLinearVelocity() {
    return new Translation2d(
            getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond)
        .rotateBy(getHeading());
  }

  public Twist2d getFieldVelocity() {
    return new Twist2d(
        getLinearVelocity().getX(),
        getLinearVelocity().getY(),
        getState().Speeds.omegaRadiansPerSecond);
  }

  public void stop() {
    setControl(new SwerveRequest.Idle());
  }

  public static final Field2d fieldWidget = new Field2d();

  @Override
  public void periodic() {
    attemptToSetPerspective();
    
    validateVisionMeasurement("limelight-front");
    fieldWidget.setRobotPose(getPose());
    SmartDashboard.putData("Field", fieldWidget);
    
    
  }

  public void attemptToSetPerspective() {
    operatorPerspectiveApplier.runOnceWhenTrueThenWhenChanged(
        () -> setOperatorPerspectiveForward(AllianceFlipUtil.getFwdHeading()),
        DriverStation.getAlliance().isPresent(),
        DriverStation.getAlliance().orElse(null));
  }

   
      private void validateVisionMeasurement(String limelight){
        LimelightHelpers.SetRobotOrientation(limelight, getHeading().getDegrees(), getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), 0, 0, 0, 0);
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

        if (poseEstimate == null
        || poseEstimate.tagCount == 0
        || !FieldConstants.FIELD_AREA.contains(poseEstimate.pose.getTranslation())
        || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 540
        || getLinearVelocity().getNorm() > 3.0) {
      return;
    }

        if(LimelightHelpers.getTV(limelight)){
          System.out.println("Limelight: I have found a target!");
        }

        addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        VecBuilder.fill(
            Math.pow(0.5, poseEstimate.tagCount) * poseEstimate.avgTagDist * 2,
            Math.pow(0.5, poseEstimate.tagCount) * poseEstimate.avgTagDist * 2,
            9999999));
    }
            
}
