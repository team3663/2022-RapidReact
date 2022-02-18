package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.drivers.Limelight;
import frc.robot.utils.Matrix;

import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private SparkMaxPIDController shooterPIDController;
  private CANSparkMax hoodMotor;
  private DigitalInput hoodLimit;
  private SparkMaxPIDController hoodPidController;
  private MotorControllerGroup shooterMotorGroup ;
  private RelativeEncoder hoodEncoder;

  private boolean running = false;

  private Limelight limelight;

  //THESE NUMBERS ARE ALL COMPLETELY WRONG
  private final int[][] KNOWN_POINTS = new int[][] { 
    { 3490, 0, 5 }, 
    { 3400, 10, 10 }, 
    { 3730, 20, 15 },
    { 3900, 30, 18 }, 
    { 4090, 40, 20 }, 
    { 4100, 30, 23 },
    { 4390, 20, 25 } 
  };

  private final int RPM_COLUMN = 0; // column index for RPM values
  private final int ANGLE_COLUMN = 1; // column index for angle values
  private final int DISTANCE_COLUMN = 2; // column index for distance values

  private final int MAX_HOOD_ANGLE = 80; //THESE NUMBERS ARE ALL COMPLETELY IMAGINARY
  private final int MIN_HOOD_ANGLE = 25; //THESE NUMBERS ARE ALL COMPLETELY IMAGINARY

  private final double WHEEL_GEAR_RATIO_MULTIPLIER = 1; //THESE NUMBERS ARE ALL COMPLETELY IMAGINARY
  private final double ANGLE_GEAR_RATIO_MULTIPLIER = 1; //THESE NUMBERS ARE ALL COMPLETELY IMAGINARY
  private final double ROTATIONS_PER_DEGREE = 5; //THESE NUMBERS ARE ALL COMPLETELY IMAGINARY

  static final double powerIncrement = 0.05; 
  public double power = 0.0;

  /** Creates a new instance of the Shooter subsystem. */
  public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitCANID) {

    shooterMotor1 = new CANSparkMax(shooterMotor1CANID, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(shooterMotor2CANID, MotorType.kBrushless);
    hoodMotor = new CANSparkMax(hoodMotorCANID, MotorType.kBrushless);
    hoodLimit = new DigitalInput(hoodLimitCANID);

    shooterMotorGroup = new MotorControllerGroup(shooterMotor1, shooterMotor2);

    hoodEncoder = hoodMotor.getEncoder();

    hoodLimit = new DigitalInput(hoodLimitCANID);

    limelight = RobotContainer.getVision();

    // The motors in the shooter run in opposition to each other by default
    // invert one of them to fix this and initialize power to zero.
    shooterMotor1.setInverted(true);
    shooterMotorGroup.set(0);
  }
  
  @Override
  public void periodic() { 
    if(limelight.getDistance() > 0){
      setRPM(calculateRPM(limelight.getDistance(), true));
    }
  }

  public void start() {
    running = true;
    shooterMotorGroup.set(power);
  }

  public void stop() {
    running = false;
    shooterMotorGroup.set(0.0);
  }

  public void increasePower() {
      power += powerIncrement;
      if (power > 1.0) {
        power = 1.0;
      }

      if ( running ) {
        shooterMotorGroup.set(power);
      }
  }

  public void decreasePower() {
    power -= powerIncrement;
    if (power < 0.0) {
      power = 0.0;
    }

    if (running) {
      shooterMotorGroup.set(power);
    }
  }

  public int calculateRPM(double distance, boolean rpm){
    if(distance < KNOWN_POINTS[ANGLE_COLUMN][DISTANCE_COLUMN]){
      return (cubicInterpolation(0, 1, 2, 3, distance, rpm));
    }
    if(distance < KNOWN_POINTS[ANGLE_COLUMN + 1][DISTANCE_COLUMN]){
      return (cubicInterpolation(0, 1, 2, 3, distance, rpm));
    }
    if(distance >= KNOWN_POINTS[KNOWN_POINTS.length - 1][DISTANCE_COLUMN]){
        return cubicInterpolation(KNOWN_POINTS.length, KNOWN_POINTS.length - 1, KNOWN_POINTS.length - 2, KNOWN_POINTS.length - 3, distance, rpm);
    }
    for(int i = 0; i < KNOWN_POINTS.length - 1; i++){
      if(distance == KNOWN_POINTS[i][DISTANCE_COLUMN]){
        if(rpm){
          return KNOWN_POINTS[i][RPM_COLUMN];
        }
        return KNOWN_POINTS[i][ANGLE_COLUMN];
      } else if(distance < KNOWN_POINTS[i+1][DISTANCE_COLUMN]){
        return (cubicInterpolation(i-2, i-1, i, i+1, distance, rpm));
      }
    }
    if(rpm){
      return KNOWN_POINTS[KNOWN_POINTS.length - 3][RPM_COLUMN];
    }
    return KNOWN_POINTS[KNOWN_POINTS.length - 3][ANGLE_COLUMN];
  }

  private int cubicInterpolation(int a, int b, int c, int d, double distance, boolean rpm){
    //solve A(distance) + B(distance) + C(distance) + D(distance) = y
    //return y

    double[] distances = {KNOWN_POINTS[a][DISTANCE_COLUMN], KNOWN_POINTS[b][DISTANCE_COLUMN], KNOWN_POINTS[c][DISTANCE_COLUMN], KNOWN_POINTS[d][DISTANCE_COLUMN], distance};
    double[] rpms = {KNOWN_POINTS[a][RPM_COLUMN], KNOWN_POINTS[b][RPM_COLUMN], KNOWN_POINTS[c][RPM_COLUMN], KNOWN_POINTS[d][RPM_COLUMN], distance};
    if(!rpm){
      rpms[0] = KNOWN_POINTS[a][ANGLE_COLUMN];
      rpms[1] = KNOWN_POINTS[b][ANGLE_COLUMN];
      rpms[2] = KNOWN_POINTS[c][ANGLE_COLUMN];
      rpms[3] = KNOWN_POINTS[d][ANGLE_COLUMN];
    }
    double[][] matrix1 = {
                        {(distances[0] * distances[0] * distances[0]), (distances[0] * distances[0]), distances[0], 1},
                        {(distances[1] * distances[1] * distances[1]), (distances[1] * distances[1]), distances[1], 1},
                        {(distances[2] * distances[2] * distances[2]), (distances[2] * distances[2]), distances[2], 1},
                        {(distances[3] * distances[3] * distances[3]), (distances[3] * distances[3]), distances[3], 1}
    };

    System.out.print(matrix1[1][1]);
    double[][] matrix2 = {
                        {rpms[0]},
                        {rpms[1]},
                        {rpms[2]},
                        {rpms[3]}
    };

    Matrix m1 = new Matrix(matrix1);
    Matrix m2 = new Matrix(matrix2);

    double[] abcd = m1.solve(m2);

    double y = abcd[0] * (distance * distance * distance) + abcd[1] * (distance * distance) + abcd[2] * distance + abcd[3];

    return (int) y;
  }

  public ShooterSubsystem resetHoodEncoder(){
    hoodEncoder.setPosition(MIN_HOOD_ANGLE);
    return this;
  }

  public void setRPM(int rpm){
    shooterPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void setHoodAngle(int degree){
    if(degree > MAX_HOOD_ANGLE){
      degree = MAX_HOOD_ANGLE;
    }
    if(degree < MIN_HOOD_ANGLE){
      degree = MIN_HOOD_ANGLE;
    }
    resetHoodEncoder();
    degree = MIN_HOOD_ANGLE;
    hoodPidController.setReference(degree * ROTATIONS_PER_DEGREE, ControlType.kPosition);
  }
}
