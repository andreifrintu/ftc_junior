package org.firstinspires.ftc.teamcode.drive.Autonom;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(group = "autonom")
public class Autonom extends LinearOpMode
{

    SampleMecanumDrive Drive;

    VuforiaBase vf;

    double maxMotorPower = 0.1;

    String VuforiaKey = "Aaiq/1//////AAABmR5nE1/0E0LclZpr6AaY5+A2o36In7uJDJ6OQngVynh2aDFKeiUTZQggihn/8KkhWmh5Jnb9cj7GU4nRu0leL6fxUJ4jg2j/4x2W+eVBwqiHHJPwMfYElGUwFiCT9CycVyk+lycCrUcMQrUMe2Aq0kWxMD3xbMDWBVUq2V3ceG6ec9GGYF/HRjVx2FoGFsiuxziwYFY/mKGN8l2kMvYvYdCog0XgHWMi5lfHo/cg0kXeVBYx72I7xD6pXuGMZlf3Lhk61R0iKn0uJ+rnZdc9UWpFhyQTokQDTCiJ5wm3eNShGn5qLSeIyw2w0wLWtLRRBlJEgxc2LOQeDjogMAIiXSvIw6pAbkRR8QflyUpNQ4j5";

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();
        waitForStart();
        while (!isStopRequested())
        {

            sleep(30000);
        }
    }

    void Init()
    {
        Drive = new SampleMecanumDrive(hardwareMap);
        Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive.setPoseEstimate(new Pose2d(0, 0));
        Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vf.initialize("", VuforiaLocalizer.CameraDirection.FRONT, true, false, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, 0, 0, 0, AxesOrder.XYZ, 0, 0, 0, true);
        vf.activate();
    }
}