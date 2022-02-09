package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;


public class PhotonVisionSubsystem extends SubsystemBase {


    // Change this to match the name of your camera

    PhotonCamera Cameron = new PhotonCamera("Cameron");
    PhotonCamera Camille = new PhotonCamera("Camille");

public void targeting(){

    var resultCameron = Cameron.getLatestResult();

    if(resultCameron.hasTargets()){
        double XOfTarget = resultCameron.getBestTarget().getPitch();
        double YOfTarget = resultCameron.getBestTarget().getYaw();
        SmartDashboard.putNumber("X of Cameron's target", XOfTarget);
        SmartDashboard.putNumber("Y of Cameron's target", YOfTarget);

        double cameraHeightMeters = 1;
        double targetHeightMeters = 1.5;
        double cameraPitchRadians = 0;
        double targetPitchRadians = 0;
        double DisToTarget = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters, cameraPitchRadians, targetPitchRadians);
        SmartDashboard.putNumber("Dis To Target", DisToTarget);
    }

    var resultCamille = Camille.getLatestResult();

    Boolean areWeRed = SmartDashboard.getBoolean("We Red Alliance", false);

    if(areWeRed){
        Camille.setPipelineIndex(2);
    }
    else{
        Camille.setPipelineIndex(1);
    }

    if(resultCamille.hasTargets()){
        double XOfTarget = resultCamille.getBestTarget().getPitch();
        double YOfTarget = resultCamille.getBestTarget().getYaw();
        
        SmartDashboard.putNumber("X of Camille's target", XOfTarget);
        SmartDashboard.putNumber("Y of Camille's target", YOfTarget);
    }

}
        
}