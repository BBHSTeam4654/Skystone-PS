package org.firstinspires.ftc.teamcode.framework.subsystems;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class SkystoneID{
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    WebcamName webcamename;

    public void intitialize(){
        //Set webcame
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
       
        //Set screen monitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //vuforia KEY
        parameters.vuforiaLicenseKey = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

        parameters.cameraName = webcamName;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables skystones = this.vuforia.loadTrackablesFromAsset(""); //FIND THE TRACKING ASSET
        
    }

    public int locate(int ID){
        int identity;
        //return map with all positional data?

        return identity;
    }


}