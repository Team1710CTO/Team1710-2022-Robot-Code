package frc.robot.commands.auto;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FasterShootInAuto extends CommandBase {

  /** Creates a new Shoot. */

  public ShooterSubsystem shooterSubsystem;
  public HoodSubsystem hoodSubsystem;
  public IndexerSubsystem indexerSubsystem;
  public PhotonVisionSubsystem photonVisionSubsystem;
  public DrivetrainSubsystem drivetrainSubsystem;
  // public PIDController rotationController;
  public final Timer timer, delayTimer;
  public boolean targetSeen = false;
  public double delay;

  public FasterShootInAuto(
    ShooterSubsystem shooterSubsystem,
    HoodSubsystem hoodSubsystem,
    IndexerSubsystem indexerSubsystem,
    PhotonVisionSubsystem photonVisionSubsystem,
    double delayOfSpinUp
  ) {
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;
    this.delay = delayOfSpinUp;

    timer = new Timer();
    delayTimer = new Timer();

    // rotationController = new PIDController(.2, .15, 0);

    addRequirements(
      shooterSubsystem,
      hoodSubsystem,
      indexerSubsystem,
      photonVisionSubsystem,
      drivetrainSubsystem
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();

    delayTimer.reset();
    delayTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // No-op until the delay has elapsed
    if (delayTimer.get() < delay) {
      return;
    }

    // Stop the timer once delay elapses.
    delayTimer.stop();

    // Distance?
    // This probably has to be scaled to match the vision systems.
    double odoDistance = DrivetrainSubsystem.m_distToCenterInMeters;
    double d = photonVisionSubsystem.getDistanceToGoalMeters(odoDistance) + 8;

    if (photonVisionSubsystem.hasGoalTargets()) {
      // Can we dummy this value since we'll be moving? (we know via the odometry how far we'll be from the target)
      if (d > 96) { // Something tells me the distance is not 96 meters away lol.
        hoodSubsystem.setHoodPosition(1.1);
        shooterSubsystem.setSpeed(10.1 * d + 2864);
      } else {
        hoodSubsystem.setHoodPosition((.0073 * d) + .388);
        shooterSubsystem.setSpeed((3700 + (-10.3 * d) + (.129 * (d * d))));
      }
      // Commented out - Let path finder take the wheel - since we are in auto
      // drivetrainSubsystem.drive(
      //   new ChassisSpeeds(
      //     0,
      //     0,
      //     -rotationController.calculate(
      //       photonVisionSubsystem.getXDisplacementOfGoal()
      //     )
      //   )
      // );
      // Commented out - lets assume path finding gets us to the right place.
      // And we'll see the target... (if not something has gone horribly wrong)
      // } else if (!targetSeen) {
      //   timer2.start();
      //   timer3.start();

      //   if (timer2.get() > .5) {
      //     drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -5));
      //   } else {
      //     drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 5));
      //   }
      // TODO: Using odometry we can guess where our scoring hub (center field).
    }

    if (shooterSubsystem.isShooterToSpeedAndNotDisabled()) {
      indexerSubsystem.runindexerInFAST();
      timer.start();
    } else {
      indexerSubsystem.stopIndexer();
      timer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.disableShooter();
    hoodSubsystem.setHoodPosition(0.1);
    indexerSubsystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Takes .3 seconds to shoot both balls?
    return timer.get() > .3;
  }
}
