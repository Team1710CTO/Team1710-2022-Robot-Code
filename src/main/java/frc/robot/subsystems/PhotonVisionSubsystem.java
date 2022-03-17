package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class PhotonVisionSubsystem extends SubsystemBase {
    // Change these to match the name of the cameras in the PhotonVision UI
    static PhotonCamera Cameron = new PhotonCamera("Cameron"); // SHOOTER CAM
    static PhotonCamera Camille = new PhotonCamera("Camille"); // INTAKE CAM

    @Override
    public void periodic() {


        var resultsCamille = Camille.getLatestResult();
        double XDisplacementOfGoal = 0;
        double YDisplacementOfGoal = 0;

        if (resultsCamille.hasTargets()) {
            
            XDisplacementOfGoal = resultsCamille.getBestTarget().getYaw();
            YDisplacementOfGoal = resultsCamille.getBestTarget().getPitch();
            SmartDashboard.putNumber("ball x displace", XDisplacementOfGoal);
            SmartDashboard.putNumber("ball y displace", YDisplacementOfGoal);
        } else {
            XDisplacementOfGoal = 0;
            YDisplacementOfGoal = 0;
            SmartDashboard.putNumber("ball x displace", XDisplacementOfGoal);
            SmartDashboard.putNumber("ball y displace", YDisplacementOfGoal);
        }
        
        getDistanceToGoalMeters(0.0);

    }

    public static double getDistanceToGoalMeters(double odometryDistance) {
        double groundDisToTarget;
        var resultCameron = Cameron.getLatestResult(); // Gets the camera's results
        if (resultCameron.hasTargets()) {
            // Distance to target calculation
            //double cameraHeightMeters = 0.7874;
            //double targetHeightMeters = 1.8161; // the actual height
            //double cameraPitch = 55; // TODO
            groundDisToTarget = resultCameron.getBestTarget().getPitch() ; // STILL TESTING
            double x = groundDisToTarget;
            double result = 105 + -8.26*x + 0.359*x*x;
            SmartDashboard.putNumber("Ground Distance To Target", result);
            return result;
             // Puts the distance to
                                                                                      // SmartDashboard
        } else {
            groundDisToTarget = odometryDistance; // Sets a default value when no targets are seen
        }
        return groundDisToTarget; // Returns the distance to the goal
    }

    public double getXDisplacementOfGoal() {
        var resultsCameron = Cameron.getLatestResult();
        double XDisplacementOfGoal = 0;
        if (resultsCameron.hasTargets()) {
            XDisplacementOfGoal = resultsCameron.getBestTarget().getYaw();
        } else {
            XDisplacementOfGoal = 0;
        }
        SmartDashboard.putNumber("xdisplacement", XDisplacementOfGoal);
        return XDisplacementOfGoal; // Returns the X displacement from the center of the camera's view to the goal
    }

    public double getYDisplacementOfGoal() {
        var resultsCameron = Cameron.getLatestResult();

        double YDisplacementOfGoal = 0;
        if (resultsCameron.hasTargets()) {
            YDisplacementOfGoal = resultsCameron.getBestTarget().getPitch();
        } else {
            YDisplacementOfGoal = 0;
        }
        return YDisplacementOfGoal; // Returns the X displacement from the center of the camera's view to the goal
    }

    public void setAlliancePipelinesRed() {
        Camille.setPipelineIndex(1); // Pipeline that tracks red balls
    }

    public void setAlliancePipelinesBlue() {
        Camille.setPipelineIndex(2); // Pipeline that tracks blue balls
    }

    public static double getDistanceToBallMeters() {
        double DisToTargetMeters = 0; // Sets a default value when no targets are seen
        var resultCamille = Camille.getLatestResult(); // Gets the camera's results
        if (resultCamille.hasTargets()) {
            // Distance to target calculation
            double cameraHeightMeters = Units.inchesToMeters(28.75); // TODO
            double targetHeightMeters = 0.075; // TODO
            double cameraPitchRadians = Units.degreesToRadians(110); // TODO
            double targetPitchRadians = Units.degreesToRadians(resultCamille.getBestTarget().getPitch());
            DisToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters,
                    cameraPitchRadians, targetPitchRadians);
            SmartDashboard.putNumber("Dis To Ball", DisToTargetMeters); // Puts the distance to SmartDashboard
        }
        return DisToTargetMeters; // Returns the distance to the best target ball
    }

    public double getXDisplacementOfBall() {
        var resultsCamille = Camille.getLatestResult();
        double XDisplacementOfBall = 0;
        if (resultsCamille.hasTargets()) {
            XDisplacementOfBall = resultsCamille.getBestTarget().getYaw();
        } else {
            XDisplacementOfBall = 0;
        }
        SmartDashboard.putNumber("xdisplacement of ball", XDisplacementOfBall);
        return XDisplacementOfBall;
    }
    public static double getYDisplacementOfBall(){
        var resultsCamille = Camille.getLatestResult();
        double YDisplacementOfBall = 0;
        if (resultsCamille.hasTargets()) {
            YDisplacementOfBall = resultsCamille.getBestTarget().getPitch();
        } else {
            YDisplacementOfBall = 0;
        }
        SmartDashboard.putNumber("Ydisplacement of ball", YDisplacementOfBall);
        return YDisplacementOfBall;
    }

    public static boolean doesIntakeSeeBall() {
        var resultsCamille = Camille.getLatestResult();
        boolean doesIntakeSeeBall = false;
        if (resultsCamille.hasTargets()) {
            doesIntakeSeeBall = true;
        }
        SmartDashboard.putBoolean("intake see", doesIntakeSeeBall);
        return doesIntakeSeeBall;
    }

    public static boolean hasGoalTargets(){

        var results = Cameron.getLatestResult();
        
        if (results.hasTargets()) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean hasBallTargets(){

        var results = Camille.getLatestResult();
        
        if (results.hasTargets()) {
            return true;
        } else {
            return false;
        }

    }
    
}