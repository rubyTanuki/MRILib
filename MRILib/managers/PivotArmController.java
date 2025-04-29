package MRILib.managers;

// Unused import: MRILib.managers.* (already in the same package)
import static MRILib.BotValues.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// Unused import: org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// Unused import: org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controls a pivoting arm using a Linear Quadratic Regulator (LQR) feedback controller
 * combined with gravity and friction feedforward terms.
 * <p>
 * This class implements {@link Runnable} to allow the control loop to run in a separate thread,
 * calculating and applying motor voltage adjustments periodically. It requires an {@link ArmBotFF}
 * instance to interact with the arm's hardware (motor, sensors, voltage).
 * <p>
 * State variables (angles, angular velocities) are primarily handled in radians.
 * The control loop aims for a consistent execution rate (default 10ms).
 *
 * @see ArmBotFF
 * @see Runnable
 * @see BotValues#TICKS_PER_DEGREE_ARM
 */
public class PivotArmController implements Runnable{
    /** The specific ArmBot instance providing hardware access. */
    private ArmBotFF bot;

    // --- Controller Gains ---
    /** LQR feedback gain for position error. */
    private final double k1 = 10.0; //LQR gain 1
    /** LQR feedback gain for velocity error. */
    private final double k2 = 3.19374388; //LQR gain 2
    /** Feedforward gain for counteracting gravity. */
    private final double kG = 1.0; //gravity gain
    /** Feedforward gain for counteracting friction (viscous). */
    private final double kD = 1.0; //friction gain

    // --- Target State (Volatile for thread safety) ---
    /** The desired target angle of the arm in radians. */
    private volatile double targetTheta; //radians
    /** The desired target angular velocity of the arm in radians per second. */
    private volatile double targetAngularVelocity; // rad/s

    // --- Current State (Volatile for thread safety) ---
    /** The current estimated angle of the arm in radians. Updated via {@link #updateCurrentState}. */
    private volatile double currentArmTheta; // radians
    /** The current estimated angular velocity of the arm in radians per second. Updated via {@link #updateCurrentState}. */
    private volatile double currentArmAngularVelocity; // rad/s

    // --- Previous State (Used for derivatives) ---
    /** The arm angle from the previous control loop iteration (radians). */
    private double previousArmTheta;
    /** The arm angular velocity from the previous control loop iteration (rad/s). */
    private double previousArmAngularVelocity;

    // --- Control Flag (Volatile for thread safety) ---
    /** Flag indicating whether the control loop thread should continue running. Set to false by {@link #stop()}. */
    private volatile boolean running = true;

    /**
     * Constructs a PivotArmController.
     *
     * @param bot The {@link ArmBotFF} instance associated with the arm being controlled.
     *            This provides access to motor control, sensors, and voltage readings.
     */
    public PivotArmController(ArmBotFF bot){
        //constructor
        this.bot = bot;
        // Initialize target state (optional, defaults to 0)
        this.targetTheta = 0.0;
        this.targetAngularVelocity = 0.0;
        // Initialize current/previous state (optional, will be updated on first updateCurrentState call)
        this.currentArmTheta = 0.0; // Or read initial position if available
        this.previousArmTheta = currentArmTheta;
        this.currentArmAngularVelocity = 0.0;
        this.previousArmAngularVelocity = currentArmAngularVelocity;
    }


    /**
     * The main control loop executed by the thread.
     * <p>
     * This method periodically calculates the required motor voltage based on the
     * target state, current state estimates, LQR feedback, and feedforward terms.
     * It runs continuously until {@link #stop()} is called.
     * The loop aims for a fixed time step (default 10ms).
     */
    @Override
    public void run(){
        long loopTime = 10; // Target loop time in milliseconds
        long loopTimeNanos = loopTime * 1_000_000; // Target loop time in nanoseconds
        long nextLoopTime = System.nanoTime() + loopTimeNanos; // Time for the next iteration

        ElapsedTime timer = new ElapsedTime();
        timer.reset(); // Ensure timer starts at 0
        double previousTime = 0.0; // Initialize previous time


        while(running){
            // --- Timekeeping ---
            double currentTime = timer.seconds();
            double deltaTime = currentTime - previousTime;
            // Prevent division by zero or huge derivatives on first loop or after pauses
            if (deltaTime <= 0) {
                deltaTime = loopTime / 1000.0; // Use target loop time if delta is invalid
            }
            previousTime = currentTime;


            // --- State Estimation (Derivatives) ---
            // Estimate angular velocity (rad/s) using finite difference
            double thetaDot = (currentArmTheta - previousArmTheta) / deltaTime;
            // Estimate angular acceleration (rad/s^2) using finite difference
            // Note: Using currentArmAngularVelocity directly might be noisy if it's derived.
            // Consider filtering or using a more robust estimation method if needed.
            // double thetaDotDot = (currentArmAngularVelocity - previousArmAngularVelocity) / deltaTime; // More accurate if velocity is reliable
            double thetaDotDot = thetaDot / deltaTime; // Simpler approximation if velocity is less reliable

            // --- Error Calculation ---
            double thetaError = targetTheta - currentArmTheta; // Position error (rad)
            double angularVelocityError = targetAngularVelocity - thetaDot; // Velocity error (rad/s)

            // --- Feedforward Calculation ---
            // Gravity compensation (depends on angle)
            double gravityFeedForward = kG * Math.cos(currentArmTheta); // Use current angle
            // Friction compensation (depends on velocity)
            double frictionFeedforward = kD * thetaDot; // Use estimated velocity
            double u_ff = gravityFeedForward + frictionFeedforward; // Total feedforward voltage

            // --- Feedback Calculation (LQR) ---
            double u_fb = (k1 * thetaError + k2 * angularVelocityError); // LQR feedback voltage (Note: Sign convention might vary based on LQR derivation)

            // --- Total Control Effort ---
            double u = u_ff + u_fb; // Combine feedforward and feedback

            // Apply calculated voltage (clamped) to the motor
            setMotorVoltage(u);

            // --- Loop Timing ---
            // Busy wait or sleep until the next loop iteration time
            while(System.nanoTime() < nextLoopTime && running){
                // Optional: Thread.yield() or short sleep to reduce CPU usage
                try {
                    // Sleep for a minimal duration to avoid pure busy-waiting
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt(); // Restore interrupt status
                    running = false; // Stop loop if interrupted
                }
            }
            // Schedule the next iteration precisely based on the target loop time
            nextLoopTime += loopTimeNanos;
        }

        // Ensure motor is stopped when the loop exits
        bot.setPivotPower(0.0);
    }

    /**
     * Converts the calculated control voltage {@code u} into a motor power value (0.0 to 1.0)
     * suitable for {@link ArmBotFF#setPivotPower(double)}, compensating for battery voltage variations.
     * The input voltage {@code u} is clamped between -12V and +12V before conversion.
     *
     * @param voltage The calculated target voltage (-12.0 to 12.0) from the controller.
     */
    public void setMotorVoltage(double voltage){
        // Clamp the calculated voltage to the physical limits (+/- 12V)
        double clampedVoltage = Math.max(-12.0, Math.min(12.0, voltage));

        // Get the current battery voltage
        double batteryVoltage = bot.getVoltage();
        // Avoid division by zero if voltage reading is invalid
        if (batteryVoltage <= 0) batteryVoltage = 12.0;

        // Calculate the motor power (-1.0 to 1.0) by scaling the clamped voltage
        // relative to the current battery voltage.
        double powerValue = clampedVoltage / batteryVoltage;

        // Apply the power value to the motor
        // Ensure power is within -1.0 to 1.0 range (should be handled by scaling, but good practice)
        bot.setPivotPower(Math.max(-1.0, Math.min(1.0, powerValue)));
    }

    /**
     * Updates the controller's internal estimate of the arm's current state (position and velocity).
     * This method should be called periodically from the main thread or sensor reading loop
     * with the latest sensor data. It converts input units (ticks, ticks/sec) to radians and rad/s.
     * <p>
     * This method is synchronized to prevent race conditions when accessed by both the main thread
     * and the controller thread.
     *
     * @param armPos   The current arm position, typically in encoder ticks.
     * @param velocity The current arm velocity, typically in encoder ticks per second.
     */
    public synchronized void updateCurrentState(double armPos, double velocity){
        // Store the previous state values before updating
        previousArmTheta = currentArmTheta;
        previousArmAngularVelocity = currentArmAngularVelocity;

        // Convert arm position from ticks to radians
        currentArmTheta = Math.toRadians(armPos / TICKS_PER_DEGREE_ARM);
        // Convert arm velocity from ticks/sec to radians/sec
        currentArmAngularVelocity = Math.toRadians(velocity / TICKS_PER_DEGREE_ARM);
    }

    /**
     * Sets the desired target angle for the arm.
     * Converts the input angle to radians if necessary.
     * Note: This implementation only sets the target angle; target velocity remains 0.
     *
     * @param theta The target angle value.
     * @param unit  The unit of the provided {@code theta} value (DEGREES or RADIANS).
     */
    public synchronized void setTargetTheta(double theta, AngleUnit unit){
        // Convert target to radians if input is in degrees
        targetTheta = (unit == AngleUnit.DEGREES) ? Math.toRadians(theta) : theta;
        // Explicitly set target velocity to zero when setting a target angle
        targetAngularVelocity = 0.0;
    }

    /**
     * Stops the control loop thread safely by setting the {@code running} flag to false.
     * The thread will finish its current iteration and then exit.
     */
    public void stop(){
        running = false;
    }
}
