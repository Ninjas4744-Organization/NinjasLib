package frc.lib.NinjasLib.localization;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.NinjasLib.swerve.Swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * A singleton background thread that samples odometry-relevant signals (CTRE Phoenix status signals
 * and arbitrary {@link DoubleSupplier}s) at a fixed frequency, higher than the main robot loop, and
 * hands out per-signal {@link Queue}s of the sampled values plus a matching timestamp queue. This lets
 * consumers (e.g. swerve modules and the gyro) build a higher-resolution odometry history than a single
 * 20&nbsp;ms periodic loop would allow, which the pose tracker can then replay through in order.
 *
 * <p>Signals must be registered with {@link #registerSignal(StatusSignal)}, {@link
 * #registerSignal(DoubleSupplier)}, and {@link #makeTimestampQueue()} before the thread is started
 * with {@link #start(int)}.
 */
public class OdometryThread extends Thread {
    private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private int odometryFrequency = 50;

    private static OdometryThread instance = null;

    /**
     * Returns the singleton {@code OdometryThread} instance, creating it on first call.
     *
     * @return The shared odometry thread instance.
     */
    public static OdometryThread getInstance() {
        if (instance == null) {
            instance = new OdometryThread();
        }
        return instance;
    }

    private OdometryThread() {
        setName("OdometryThread");
        setDaemon(true);
    }

    /**
     * Starts the background sampling thread at the given frequency. Does nothing if no signals or
     * timestamp queues have been registered yet, since there would be nothing to sample.
     *
     * @param odometryFrequency The rate, in Hz, at which to sample all registered signals.
     */
    public void start(int odometryFrequency) {
        if (!timestampQueues.isEmpty()) {
            System.out.println("[Odometry Thread] Starting on " + odometryFrequency + "Hz");
            super.start();
            this.odometryFrequency = odometryFrequency;
        }
    }

    /**
     * Registers a CTRE Phoenix status signal to be batch-refreshed and sampled by the thread every
     * cycle, alongside all other registered Phoenix signals.
     *
     * @param signal The Phoenix status signal to sample (e.g. a motor position signal).
     * @return A queue that fills with one sampled value per thread cycle; drain it periodically.
     */
    public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
        System.out.println("[Odometry Thread] Making a phoenix queue");
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        Swerve.odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            Swerve.odometryLock.unlock();
        }
        return queue;
    }

    /**
     * Registers a non-Phoenix signal (e.g. a navX gyro reading) to be sampled by the thread every
     * cycle. Unlike Phoenix signals, this is polled directly rather than batch-refreshed.
     *
     * @param signal Supplier for the value to sample each cycle.
     * @return A queue that fills with one sampled value per thread cycle; drain it periodically.
     */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        System.out.println("[Odometry Thread] Making a generic queue");
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        Swerve.odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            signalsLock.unlock();
            Swerve.odometryLock.unlock();
        }
        return queue;
    }

    /**
     * Creates a queue that fills with the sample timestamp (FPGA time, adjusted for average CAN
     * latency) for each thread cycle, matched one-to-one with the values in any signal queue registered
     * around the same time.
     *
     * @return A queue of per-cycle sample timestamps, in seconds.
     */
    public Queue<Double> makeTimestampQueue() {
        System.out.println("[Odometry Thread] Making a timestamp queue");
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Swerve.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Swerve.odometryLock.unlock();
        }
        return queue;
    }

    /**
     * The thread body: repeatedly waits for all registered Phoenix signals to update (or sleeps at the
     * configured frequency if there are none), then samples every registered signal and timestamp into
     * its queue. Not intended to be called directly; use {@link #start(int)}.
     */
    @Override
    public void run() {
        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(2.0 / odometryFrequency, phoenixSignals);
                } else {
                    // "waitForAll" does not support blocking on multiple signals with a bus
                    // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                    // behavior is provided by the documentation.
                    Thread.sleep((long) (1000.0 / odometryFrequency));
                    if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            Swerve.odometryLock.lock();
            try {
                // Sample timestamp is current FPGA time minus average CAN latency
                //     Default timestamps from Phoenix are NOT compatible with
                //     FPGA timestamps, this solution is imperfect but close
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }

                // Add new samples to queues
                for (int i = 0; i < phoenixSignals.length; i++) {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            } finally {
                Swerve.odometryLock.unlock();
            }
        }
    }
}