import org.firstinspires.ftc.teamcode.framework.Datalog;
import org.firstinspires.ftc.robotcore.external.State;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

@TeleOp(name="Encoder Counter")
public class Counter extends OpMode{
    private List<DcMotor> motors;
    private List<DcMotor> encoders;
    private IIMU imu;

    private Datalog data;
}

