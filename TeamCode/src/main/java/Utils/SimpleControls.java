package Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class SimpleControls
{
// Drive


    public void SetDCPos(DcMotor Lift, int pos, double power)
    {
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setTargetPosition(pos * -1);
        Lift.setPower(power);
    }

// Servo
    public void SetServo(Servo servo,boolean open)
    {
        servo.setPosition(open ? 0.4 : 0.0);
    }
}
