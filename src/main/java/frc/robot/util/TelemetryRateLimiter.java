package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;

/**
 * Utility class to rate-limit telemetry updates and detect significant changes.
 * Uses both time-based rate limiting (e.g., 10Hz) and change detection
 * to reduce network bandwidth by only publishing when values meaningfully change.
 */
public class TelemetryRateLimiter {
  private final double m_periodSeconds;
  private double m_lastUpdateTime;
  
  // Track previous values for change detection
  private final Map<String, Double> m_previousDoubleValues = new HashMap<>();
  private final Map<String, Boolean> m_previousBooleanValues = new HashMap<>();
  private final Map<String, String> m_previousStringValues = new HashMap<>();
  
  private final double m_changeThreshold;

  /**
   * Creates a new TelemetryRateLimiter with default change threshold (0.01).
   *
   * @param frequencyHz the desired update frequency in Hz (e.g., 10 for 10Hz updates)
   */
  public TelemetryRateLimiter(double frequencyHz) {
    this(frequencyHz, 0.01);
  }

  /**
   * Creates a new TelemetryRateLimiter with custom change threshold.
   *
   * @param frequencyHz the desired update frequency in Hz (e.g., 10 for 10Hz updates)
   * @param changeThreshold minimum absolute change to trigger an update for numbers
   */
  public TelemetryRateLimiter(double frequencyHz, double changeThreshold) {
    m_periodSeconds = 1.0 / frequencyHz;
    m_changeThreshold = changeThreshold;
    m_lastUpdateTime = -m_periodSeconds; // Allow first update immediately
  }

  /**
   * Checks if enough time has passed to publish the next update.
   *
   * @return true if the update should be published, false otherwise
   */
  public boolean tryUpdate() {
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - m_lastUpdateTime >= m_periodSeconds) {
      m_lastUpdateTime = currentTime;
      return true;
    }
    return false;
  }

  /**
   * Checks if a number has changed significantly since last update.
   * Also respects the time-based rate limit.
   *
   * @param key identifier for this value (e.g., "Launcher/RPM")
   * @param value current value
   * @return true if the value changed by more than threshold or it's time for a periodic update
   */
  public boolean hasChangedNumber(String key, double value) {
    // Always respect time-based rate limit
    if (!tryUpdate()) {
      return false;
    }

    Double previousValue = m_previousDoubleValues.get(key);
    if (previousValue == null) {
      // First time seeing this key
      m_previousDoubleValues.put(key, value);
      return true;
    }

    // Check if change is significant
    if (Math.abs(value - previousValue) > m_changeThreshold) {
      m_previousDoubleValues.put(key, value);
      return true;
    }

    return false;
  }

  /**
   * Checks if a boolean has changed.
   * Also respects the time-based rate limit.
   *
   * @param key identifier for this value
   * @param value current value
   * @return true if the value changed or it's time for a periodic update
   */
  public boolean hasChangedBoolean(String key, boolean value) {
    // Always respect time-based rate limit
    if (!tryUpdate()) {
      return false;
    }

    Boolean previousValue = m_previousBooleanValues.get(key);
    if (previousValue == null) {
      // First time seeing this key
      m_previousBooleanValues.put(key, value);
      return true;
    }

    // Booleans always update on every rate-limited cycle
    if (value != previousValue) {
      m_previousBooleanValues.put(key, value);
      return true;
    }

    return false;
  }

  /**
   * Checks if a string has changed.
   * Also respects the time-based rate limit.
   *
   * @param key identifier for this value
   * @param value current value
   * @return true if the value changed or it's time for a periodic update
   */
  public boolean hasChangedString(String key, String value) {
    // Always respect time-based rate limit
    if (!tryUpdate()) {
      return false;
    }

    String previousValue = m_previousStringValues.get(key);
    if (previousValue == null) {
      // First time seeing this key
      m_previousStringValues.put(key, value);
      return true;
    }

    if (!value.equals(previousValue)) {
      m_previousStringValues.put(key, value);
      return true;
    }

    return false;
  }

  /**
   * Resets the rate limiter, allowing the next update immediately.
   */
  public void reset() {
    m_lastUpdateTime = -m_periodSeconds;
    m_previousDoubleValues.clear();
    m_previousBooleanValues.clear();
    m_previousStringValues.clear();
  }
}
