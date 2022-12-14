package org.firstinspires.ftc.teamcode.drive.Autonom;


import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.internal.system.ClassFactoryImpl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(group = "autonom")
public class CamTest extends LinearOpMode
{

    SampleMecanumDrive Drive;

    VuforiaBase vf;

    double maxMotorPower = 0.1;

    String vuforiaKey = "Aaiq/1//////AAABmR5nE1/0E0LclZpr6AaY5+A2o36In7uJDJ6OQngVynh2aDFKeiUTZQggihn/8KkhWmh5Jnb9cj7GU4nRu0leL6fxUJ4jg2j/4x2W+eVBwqiHHJPwMfYElGUwFiCT9CycVyk+lycCrUcMQrUMe2Aq0kWxMD3xbMDWBVUq2V3ceG6ec9GGYF/HRjVx2FoGFsiuxziwYFY/mKGN8l2kMvYvYdCog0XgHWMi5lfHo/cg0kXeVBYx72I7xD6pXuGMZlf3Lhk61R0iKn0uJ+rnZdc9UWpFhyQTokQDTCiJ5wm3eNShGn5qLSeIyw2w0wLWtLRRBlJEgxc2LOQeDjogMAIiXSvIw6pAbkRR8QflyUpNQ4j5";

    VuforiaLocalizer vuforia;


    private VuforiaTrackables signalTargets;
    private VuforiaTrackableDefaultListener signalListener;
    private WebcamName webcamName;

    ClassFactory classFactory = new ClassFactoryImpl(); // idk

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = vuforiaKey;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // dubios dubios dubios
        vuforia = classFactory.getInstance().createVuforia(params);
        signalTargets = vuforia.loadTrackablesFromAsset("ConuriNoi");
        signalTargets.get(0).setName("1");
        signalTargets.get(1).setName("2");
        signalTargets.get(2).setName("3");
        signalTargets.activate();

        waitForStart();


        // https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/using-vumarks.pdf
        // https://www.youtube.com/watch?v=2z-o9Ts8XoE

        while (!isStopRequested()) {

            Bitmap beta = Bitmap.createBitmap(vuforia.rgb, );

            for (VuforiaTrackable signal : signalTargets) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) signal.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    double degreeToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(signal.getName() + "-deg", degreeToTurn);
                }
                telemetry.update();
            }

        }
    }

    public void Init()
    {
        Drive = new SampleMecanumDrive(hardwareMap);
        Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive.setPoseEstimate(new Pose2d(0, 0));
        Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}