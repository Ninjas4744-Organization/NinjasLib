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

public class OdometryThread extends Thread {
    private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private int odometryFrequency = 50;

    private static OdometryThread instance = null;

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

    public void start(int odometryFrequency) {
        if (!timestampQueues.isEmpty()) {
            super.start();
            this.odometryFrequency = odometryFrequency;
        }
    }

    /**
     * Registers a Phoenix signal to be read from the thread.
     */
    public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
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
     * Registers a generic signal to be read from the thread.
     */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
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
     * Returns a new queue that returns timestamp values for each sample.
     */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Swerve.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Swerve.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (phoenixSignals.length > 0) {
//                    System.out.println("Phoenix signals: " + phoenixSignals.length);
//                    System.out.println("generic signals: " + genericSignals.size());
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