package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import drivers.PhoenixSwerveHelper;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.PoseEstimate;
import lombok.Getter;
import java.util.function.Supplier;
import java.util.logging.Logger;
import util.AllianceFlipUtil;
import util.RunnableUtil.RunOnce;
import vision.LimelightVision.Limelight;
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

  public void addVisionPose(String limelight){
    if(LimelightHelpers.getTV(limelight) 
    && LimelightHelpers.getTA(limelight) >= 0.5 
    && FieldConstants.FIELD_AREA.contains(m_PoseEstimate.getEstimatedPosition().getTranslation())){
      m_PoseEstimate.addVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight).pose, LimelightHelpers.getLatency_Capture(limelight));
    }
  }
  public static final Field2d fieldWidget = new Field2d();

  @Override
  public void periodic() {
    attemptToSetPerspective();

    m_PoseEstimate.update(getPigeon2().getRotation2d(), getState().ModulePositions);
   // validateVisionMeasurement("limelight-front",LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front") , m_PoseEstimate);
    fieldWidget.setRobotPose(m_PoseEstimate.getEstimatedPosition());

    addVisionPose("limelight-front");
    
    fieldWidget.setRobotPose(m_PoseEstimate.getEstimatedPosition());
    SmartDashboard.putNumber("Estimate Pose X", m_PoseEstimate.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Estimate Pose Y", m_PoseEstimate.getEstimatedPosition().getY());
    SmartDashboard.putData("Field", fieldWidget);
   
    
  }

  public void attemptToSetPerspective() {
    operatorPerspectiveApplier.runOnceWhenTrueThenWhenChanged(
        () -> setOperatorPerspectiveForward(AllianceFlipUtil.getFwdHeading()),
        DriverStation.getAlliance().isPresent(),
        DriverStation.getAlliance().orElse(null));
  }

      private static void validateVisionMeasurement(String limelight, PoseEstimate estimate, SwerveDrivePoseEstimator poseEstimator){
        if (estimate.pose.getX() == 0.0 || !FieldConstants.FIELD_AREA.contains(estimate.pose.getTranslation())) {
          return;
        }

        double latency = LimelightHelpers.getLatency_Pipeline(limelight) + LimelightHelpers.getLatency_Capture(limelight);
        latency = latency / 1000.0;

        double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
            .getDistance(estimate.pose.getTranslation());

        if (estimate.tagCount != 0) {
            double xyStds;
            if (estimate.tagCount >= 2) {
                xyStds = 0.3;
            } else if (estimate.avgTagArea > 0.8 && poseDifference < 0.3) {
                xyStds = 0.7;
            } else if (estimate.avgTagArea < 0.1 && poseDifference < 0.9) {
                xyStds = 0.7;
            } else {
                return;
            }
            poseEstimator.addVisionMeasurement(estimate.pose, Timer.getFPGATimestamp() - latency, VecBuilder.fill(xyStds, xyStds, 99999999));
        }
    }
}
