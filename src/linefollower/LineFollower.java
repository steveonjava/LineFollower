package linefollower;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
import com.pi4j.io.serial.Serial;
import com.pi4j.io.serial.SerialDataEvent;
import com.pi4j.io.serial.SerialDataListener;
import com.pi4j.io.serial.SerialFactory;
import com.pi4j.wiringpi.Gpio;
import com.pi4j.wiringpi.SoftPwm;
import java.io.IOException;

/**
 *
 * @author Steve
 */
public class LineFollower {
    public static enum Drive {reverse, stop, forward, left, right};
    public static enum Speed {
        slow(55), medium(65), fast(80), accelerate(100);
        private final int pulse;
        Speed(int pulse) {
            this.pulse = pulse;
        }
        public int getPulse() {
            return pulse;
        }
        public Speed speedup() {
            switch (this) {
                case slow: return medium;
                case medium: return fast;
                case fast: return accelerate;
                case accelerate: return accelerate;
                default: throw new IllegalStateException("Unknown speedup " + this);
            }
        }
        public Speed slowdown() {
            switch (this) {
                case accelerate: return fast;
                case fast: return medium;
                case medium: return slow;
                case slow: return slow;
                default: throw new IllegalStateException("Unknown slowdown " + this);
            }
        }
    };
    private final GpioController gpio;
//    private final I2CDevice a2d;
    private final GpioPinDigitalOutput motor1Dir;
    private final GpioPinDigitalOutput motor2Dir;
    private final GpioPinDigitalInput lineFollowA;
    private final GpioPinDigitalInput lineFollowB;
    private final GpioPinDigitalOutput gripperDir;
    private final Serial remote;
    private Drive drive;
    private Speed speed = Speed.medium;
    private boolean followLine;
    private Drive lineLocation = Drive.forward;
    private boolean gripOpen = true;
    
    public static void main(String[] args) throws IOException, InterruptedException {
        LineFollower lineFollower = new LineFollower();
        lineFollower.run();
    }

    public LineFollower() throws IOException {
        gpio = GpioFactory.getInstance();
//        I2CBus i2CBus = I2CFactory.getInstance(1);
//        a2d = i2CBus.getDevice(0x08);
        // Motor 1: dir=0 / pulse=7
        // Motor 2: dir=5 / pulse=6
        // Port 1 (motor 3): dir=10 / pulse=12
        // Port 2 (motor 4): dir=3 / pulse=4
        // Port 3: GPIO 13 / 14
        // Port 4: GPIO 2 / 1
        motor1Dir = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_00, "Motor1Dir", PinState.HIGH);
        motor2Dir = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_05, "Motor2Dir", PinState.HIGH);
        gripperDir = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_10, "GripperDir", PinState.HIGH);
        lineFollowA = gpio.provisionDigitalInputPin(RaspiPin.GPIO_13, "LineFollowA");
        lineFollowB = gpio.provisionDigitalInputPin(RaspiPin.GPIO_14, "LineFollowA");
        startGPIO();
        remote = SerialFactory.createInstance();
        remoteListenerSetup();
    }

    private void startGPIO() throws IOException {
        int result = Gpio.wiringPiSetup();
        if (result == -1) throw new IOException("Failed to setup Wiring Pi");
        result = SoftPwm.softPwmCreate(7,0,100);
        if (result == -1) throw new IOException("Failed to create soft PWM on Pin 7");
        result = SoftPwm.softPwmCreate(6,0,100);
        if (result == -1) throw new IOException("Failed to create soft PWM on Pin 6");
        result = SoftPwm.softPwmCreate(12,0,100);
        if (result == -1) throw new IOException("Failed to create soft PWM on Pin 12");
    }

    private void remoteListenerSetup() {
        remote.addListener((SerialDataListener) (SerialDataEvent event) -> {
            String data = event.getData();
            byte command = (byte) data.charAt(2);
            switch (command) {
                case 69: // power button
                    doDrive(Drive.stop);
                    shutdown();
                    System.exit(0);
                    break;
                case 64: // plus
                    System.out.println("Speed Up!");
                    changeSpeed(speed.speedup());
                    break;
                case 7: // rewind
                    if (drive != Drive.left) {
                        doDrive(Drive.left);
                    } else {
                        doDrive(Drive.stop);
                    }
                    break;
                case 9: // fast forward
                    if (drive != Drive.right) {
                        doDrive(Drive.right);
                    } else {
                        doDrive(Drive.stop);
                    }
                    break;
                case 25: // minus
                    System.out.println("Slow Down!");
                    changeSpeed(speed.slowdown());
                    break;
                case 21: // play
                    if (drive != Drive.forward) {
                        doDrive(Drive.forward);
                    } else {
                        doDrive(Drive.stop);
                    }
                    break;
                case 67: // return
                    if (drive != Drive.reverse) {
                        doDrive(Drive.reverse);
                    } else {
                        doDrive(Drive.stop);
                    }
                    break;
                case 13: // clear
                    toggleGripper();
                    break;
                case 68: // test
                    if (!followLine) {
                        System.out.println("Initiating line following algorithm");
                        followLine = true;
                        lineLocation = Drive.forward;
                        doDrive(Drive.forward);
                    } else {
                        System.out.println("Terminating line following algorithm");
                        followLine = false;
                        doDrive(Drive.stop);
                    }
                    break;
                case 71: // menu
                case 22: // 0
                case 12: // 1
                case 24: // 2
                case 94: // 3
                case 8: // 4
                case 28: // 5
                case 90: // 6
                case 66: // 7
                case 82: // 8
                case 74: // 9
                    break; // do nothing
                default:
                    System.out.println("Unrecognized Command: " + command);
            }            
        });
        remote.open(Serial.DEFAULT_COM_PORT, 9600);
    }

    private void toggleGripper() {
        System.out.println("Grip " + (gripOpen ? "Close!" : "Open!"));
        gripperDir.setState(gripOpen ? PinState.HIGH : PinState.LOW);
        SoftPwm.softPwmWrite(12, 100);
        Gpio.delay(2650);
        SoftPwm.softPwmWrite(12, 0);
        System.out.println("Grip Done!");
        gripOpen = !gripOpen;
    }
    
    private void changeSpeed(Speed speed) {
        this.speed = speed;
        doDrive(drive);
    }
    
    private void doDrive(Drive drive) {
        switch (drive) {
            case stop:
                System.out.println("Stop!");
                SoftPwm.softPwmWrite(7,0);
                SoftPwm.softPwmWrite(6,0);
                break;
            case left:
                System.out.println("Left!");
                motor1Dir.setState(PinState.LOW);
                motor2Dir.setState(PinState.LOW);
                SoftPwm.softPwmWrite(7, speed.getPulse());
                SoftPwm.softPwmWrite(6, speed.getPulse());
                break;
            case right:
                System.out.println("Right!");
                motor1Dir.setState(PinState.HIGH);
                motor2Dir.setState(PinState.HIGH);
                SoftPwm.softPwmWrite(7, speed.getPulse());
                SoftPwm.softPwmWrite(6, speed.getPulse());
                break;
            case forward:
                System.out.println("Forward!");
                motor1Dir.setState(PinState.LOW);
                motor2Dir.setState(PinState.HIGH);
                SoftPwm.softPwmWrite(7, speed.getPulse());
                SoftPwm.softPwmWrite(6, speed.getPulse());
                break;
            case reverse:
                System.out.println("Reverse!");
                motor1Dir.setState(PinState.HIGH);
                motor2Dir.setState(PinState.LOW);
                SoftPwm.softPwmWrite(7, speed.getPulse());
                SoftPwm.softPwmWrite(6, speed.getPulse());
                break;
        }
        this.drive = drive;
    }

    public void remoteTest() throws InterruptedException {
        for (;;) {
            // wait 1 second before continuing
            Thread.sleep(1000);
        }
    }

    public void lineFollowerTest() throws IOException {
        for (int i=0; i<10000; i++) {
            System.out.println("lineFollowA.getState().isHigh() = " + lineFollowA.getState().isHigh());
            System.out.println("lineFollowB.getState().isHigh() = " + lineFollowB.getState().isHigh());
            Gpio.delay(1);
        }
    }
        
    public void motorTest() throws IOException {
        System.out.println("forward");
        motor1Dir.setState(PinState.HIGH);
        motor2Dir.setState(PinState.LOW);
        SoftPwm.softPwmWrite(7,100);
        SoftPwm.softPwmWrite(6,100);
        Gpio.delay(2000);
        System.out.println("back");
        motor1Dir.setState(PinState.LOW);
        motor2Dir.setState(PinState.HIGH);
        SoftPwm.softPwmWrite(7,100);
        SoftPwm.softPwmWrite(6,100);
        Gpio.delay(2000);
        System.out.println("spin");
        motor1Dir.setState(PinState.HIGH);
        motor2Dir.setState(PinState.HIGH);
        SoftPwm.softPwmWrite(7,100);
        SoftPwm.softPwmWrite(6,100);
        Gpio.delay(2000);
        System.out.println("stop");
        SoftPwm.softPwmWrite(7,0);
        SoftPwm.softPwmWrite(6,0);
        shutdown();
    }
    
    public void run() {
        for (;;) {
            if (followLine) {
                boolean leftSensor = lineFollowA.getState().isHigh();
                boolean rightSensor = lineFollowB.getState().isHigh();
                if (leftSensor && rightSensor) { // we are lost
                    doDrive(lineLocation);
                } else if (!leftSensor && !rightSensor) { // on the line
                    doDrive(Drive.forward);
                    lineLocation = Drive.forward;
                } else if (!leftSensor && rightSensor) { // slipping off the right
                    lineLocation = Drive.left;
                    doDrive(Drive.forward);
                } else if (leftSensor && !rightSensor) { // slipping off the left
                    lineLocation = Drive.right;
                    doDrive(Drive.forward);
                }
                Gpio.delay(1);
            } else {
                Gpio.delay(1000);
            }
        }
    }

    public void shutdown() {
        if (!gripOpen) {
            toggleGripper();
        }
        gpio.shutdown();
    }
    
}
