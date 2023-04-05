package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.JoystickButton;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private static final int leftfrontID = 1;
    private static final int leftrearID = 2;
    private static final int rightfrontID = 3;
    private static final int rightrearID = 4;
    private static final int clawID = 5;
    private static final int armID = 6;
    private static final int shoulderID = 7;

    private XboxController controller;
    private JoystickButton armButton, shoulderButton, clawButton;

    private CANSparkMax leftfront;
    private CANSparkMax leftrear;
    private CANSparkMax rightfront;
    private CANSparkMax rightrear;
    private CANSparkMax claw;
    private CANSparkMax arm;
    private CANSparkMax shoulder;

    private MecanumDrive m_robotDrive;
    private final AnalogGyro m_gyro = new AnalogGyro(0);
    private final Joystick m_joystick = new Joystick(0);

    private RelativeEncoder en_rightfront;
    private RelativeEncoder en_rightrear;
    private RelativeEncoder en_leftfront;
    private RelativeEncoder en_leftrear;
    private RelativeEncoder en_claw;
    private RelativeEncoder en_arm;
    private RelativeEncoder en_shoulder;

    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private static final int kDoubleSolenoidForward = 7;
    private static final int kDoubleSolenoidReverse = 5;
    private static final int buttonarm = 4;
    private static final int clawop = 12;
    private static final int clawclo = 11;
    private static final int shoulop = 10;
    private static final int shoulclo = 9;
    private static final int armop = 8;
    private static final int armclo = 7;

    


    private static final double kVoltsPerDegreePerSecond = 0.0128;

    @Override
    public void robotInit() {
        leftfront = new CANSparkMax(leftfrontID, MotorType.kBrushless);
        leftrear = new CANSparkMax(leftrearID, MotorType.kBrushless);
        rightfront = new CANSparkMax(rightfrontID, MotorType.kBrushless);
        rightrear = new CANSparkMax(rightrearID, MotorType.kBrushless);
        claw= new CANSparkMax(clawID, MotorType.kBrushless);
        arm= new CANSparkMax(armID, MotorType.kBrushless);
        shoulder= new CANSparkMax(shoulderID, MotorType.kBrushless);

        controller = new XboxController(0);
        armButton = new JoystickButton(controller, XboxController.Button.kA.value);
        shoulderButton = new JoystickButton(controller, XboxController.Button.kB.value);
        clawButton = new JoystickButton(controller, XboxController.Button.kX.value);
        armButton0 = new JoystickButton(controller, XboxController.Button.kA.value);
        shoulderButton0 = new JoystickButton(controller, XboxController.Button.kA.value);
        clawButton0 = new JoystickButton(controller, XboxController.Button.kA.value);


        rightfront.setInverted(true);
        rightrear.setInverted(true);

        en_leftfront = leftfront.getEncoder();
        en_leftrear = leftrear.getEncoder();
        en_rightfront = rightfront.getEncoder();
        en_rightrear = rightrear.getEncoder();
        en_claw = claw.getEncoder();
        en_arm = arm.getEncoder();
        en_shoulder = shoulder.getEncoder();

        armButton.whenPressed(new MoveArmCommand());
        shoulderButton.whenPressed(new MoveShoulderCommand());
        clawButton.whenPressed(new OpenClawCommand());
        armButton.whenPressed(new MoveArmCommand());
        armButton.whenPressed(new MoveArmCommand());



        m_robotDrive = new MecanumDrive(leftfront, leftrear, rightfront, rightrear);

        m_gyro.setSensitivity(kVoltsPerDegreePerSecond);
        m_gyro.calibrate();
    }

    @Override
    public void teleopPeriodic() {
        if (m_joystick.getRawButton(kDoubleSolenoidForward)) {
            m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        } else if (m_joystick.getRawButton(kDoubleSolenoidReverse)) {
            m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        double rightfrontencoder = en_rightfront.getPosition();

        if (m_joystick.getRawButton(buttonarm)) {
          if (rightfrontencoder <= 30.0) {
            rightfront.set(0.1); // set the power to 0.5 if encoder value is less than 3.0
            
          }

          System.out.println("Encoder value for arm: ");

        
        if (rightfrontencoder > 30.0) {
           
          rightfront.set(0.0);
        }}
        if (m_joystick.getRawButtonPressed(clawop)) {
          claw.set(0.2);
      } 
        if (m_joystick.getRawButtonPressed(clawclo)) {
          claw.set(-0.2);
      }         
        if (m_joystick.getRawButtonPressed(shoulop)) {
          shoulder.set(0.2);
      }        
        if (m_joystick.getRawButtonPressed(shoulclo)) {
          shoulder.set(-0.2);
      }         
        if (m_joystick.getRawButtonPressed(armop)) {
          arm.set(0.2);
      }         
        if (m_joystick.getRawButtonPressed(armclo)) {
          arm.set(-0.2);
      }        
      
//********----- */

        if (m_joystick.getRawButtonReleased(clawop)) {
          claw.set(0.0);
      } 
        if (m_joystick.getRawButtonReleased(clawclo)) {
          claw.set(-0.0);
      }         
        if (m_joystick.getRawButtonReleased(shoulop)) {
          shoulder.set(0.0);
      }        
        if (m_joystick.getRawButtonReleased(shoulclo)) {
          shoulder.set(-0.0);
      }         
        if (m_joystick.getRawButtonReleased(armop)) {
          arm.set(0.0);
      }         
        if (m_joystick.getRawButtonReleased(armclo)) {
          arm.set(-0.0);
      }         
      if (m_joystick.getRawButton(buttonarm)) {
          
            if (en_arm.getPosition() < 18) {
                    arm.set(0.5); // set the power to rotate the motor at half speed
                } else {
                    arm.set(0.0); // stop the motor when the arm has rotated 18 times
                }
            
            }
        }
            
          

    

    
    {



        SmartDashboard.putNumber("Encoder Position For Right Front", en_rightfront.getPosition());
        SmartDashboard.putNumber("Encoder Position For Right Rear", en_rightrear.getPosition());
        SmartDashboard.putNumber("Encoder Position For Left Front", en_leftfront.getPosition());
        SmartDashboard.putNumber("Encoder Position For Left Rear", en_leftrear.getPosition());
        SmartDashboard.putNumber("Encoder Position For claw", en_claw.getPosition());
        SmartDashboard.putNumber("Encoder Position For arm", en_arm.getPosition());
        SmartDashboard.putNumber("Encoder Position For shoulder", en_shoulder.getPosition());

        m_robotDrive.driveCartesian(
            -m_joystick.getY(), -m_joystick.getX(), -m_joystick.getZ(), m_gyro.getRotation2d());
    }




}
