package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="Iterative Opmode")
public class chatgbtTOP extends OpMode
{
    // Declare motors
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void init() {
        // Initialize motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Set motor directions
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Get joystick values
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        // Set motor power
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}

/*

import com.vuforia.*;

public class ConeTracker {
  // Vuforia initialization parameters
  private static final String VUFORIA_LICENSE_KEY = "YOUR_LICENSE_KEY_HERE";
  private static final String DATABASE_NAME = "Cones";
  private static final String TARGET_NAME = "Cone";

  // Camera parameters
  private static final int CAMERA_WIDTH = 720;
  private static final int CAMERA_HEIGHT = 480;

  // Vuforia objects
  private VuforiaLocalizer vuforia;
  private VuforiaTrackables targets;
  private VuforiaTrackable target;

  // Constructor
  public ConeTracker() {
    // Initialize Vuforia
    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    params.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
    params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
    params.cameraResolution = new Size(CAMERA_WIDTH, CAMERA_HEIGHT);
    vuforia = ClassFactory.getInstance().createVuforia(params);

    // Load the cone target
    targets = vuforia.loadTrackablesFromAsset(DATABASE_NAME);
    target = targets.get(TARGET_NAME);
  }

  // Start tracking
  public void startTracking() {
    targets.activate();
  }

  // Stop tracking
  public void stopTracking() {
    targets.deactivate();
  }

  // Check if the cone is currently being tracked
  public boolean isTracking() {
    // Get the current pose of the cone target
    Pose pose = target.getPose();
    // Check if the pose is valid
    return pose.isValid();
  }

  // Get the current position of the cone
  public Vec3F getPosition() {
    // Get the current pose of the cone target
    Pose pose = target.getPose();
    // Get the position from the pose
    Vec3F position = pose.getTranslation();
    // Return the position
    return position;
  }
}

 */