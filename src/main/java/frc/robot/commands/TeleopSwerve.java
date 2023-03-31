package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.autos.segmentLineUp;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier autoCenter;
  private BooleanSupplier robotCentricSup;
  private Double speed;
  private Double angularVelocity;


  private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0);

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier autoCenter,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.autoCenter = autoCenter;
    this.robotCentricSup = robotCentricSup;
    this.speed = Constants.Swerve.mediumSpeed;
    this.angularVelocity = Constants.Swerve.mediumAngularVelocity;
  }

  public void slowMode(){
    this.speed = Constants.Swerve.slowSpeed;
    this.angularVelocity = Constants.Swerve.slowAngularVelocity;
  }  
  public void MediumMode(){
    this.speed = Constants.Swerve.mediumSpeed;
    this.angularVelocity = Constants.Swerve.mediumAngularVelocity;
  }  

  public void fastMode(){
    this.speed = Constants.Swerve.maxSpeed;
    this.angularVelocity = Constants.Swerve.maxAngularVelocity;
  }


  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = translationLimiter
        .calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal = strafeLimiter
        .calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal = rotationLimiter
        .calculate(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

    /* Drive */
    if(autoCenter.getAsBoolean()){
      // pseudo-code
      // 1. find fiduciary target
      // 2. calculate transpose to get centers to the target

      
      // segmentLineUp.getTrajectory(Constants.SEGMENT.CUBE_3, () -> s_Swerve.getPose());
      // segmentLineUp lineup = new segmentLineUp(s_Swerve, Constants.SEGMENT.CUBE_3, () -> s_Swerve.getPoint());
      // lineup.schedule();
    }

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(speed),
        rotationVal * angularVelocity,
        !robotCentricSup.getAsBoolean(),
        false);
  }
}
