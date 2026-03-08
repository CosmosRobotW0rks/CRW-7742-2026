package frc.robot.utils;

import java.util.Comparator;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpolatingTreeMap<K, V> {
  private final TreeMap<K, V> m_map;

  private final InverseInterpolator<K> m_inverseInterpolator;
  private final Interpolator<V> m_interpolator;

  /**
   * Constructs an InterpolatingTreeMap.
   *
   * @param inverseInterpolator Function to use for inverse interpolation of the keys.
   * @param interpolator Function to use for interpolation of the values.
   */
  public InterpolatingTreeMap(
      InverseInterpolator<K> inverseInterpolator, Interpolator<V> interpolator) {
    m_map = new TreeMap<>();
    m_inverseInterpolator = inverseInterpolator;
    m_interpolator = interpolator;
  }

  /**
   * Constructs an InterpolatingTreeMap using {@code comparator}.
   *
   * @param inverseInterpolator Function to use for inverse interpolation of the keys.
   * @param interpolator Function to use for interpolation of the values.
   * @param comparator Comparator to use on keys.
   */
  public InterpolatingTreeMap(
      InverseInterpolator<K> inverseInterpolator,
      Interpolator<V> interpolator,
      Comparator<K> comparator) {
    m_inverseInterpolator = inverseInterpolator;
    m_interpolator = interpolator;
    m_map = new TreeMap<>(comparator);
  }

  /**
   * Inserts a key-value pair.
   *
   * @param key The key.
   * @param value The value.
   */
  public void put(K key, V value) {
    m_map.put(key, value);
  }

  /** Removes the entry for the given key. */
  public V remove(K key) {
    return m_map.remove(key);
  }

  public boolean remove(K key, V value) {
    return m_map.remove(key, value);
  }

  /**
   * Returns the value associated with a given key.
   *
   * <p>If there's no matching key, the value returned will be an interpolation between the keys
   * before and after the provided one, using the {@link Interpolator} and {@link
   * InverseInterpolator} provided.
   *
   * @param key The key.
   * @return The value associated with the given key.
   */
  public V get(K key) {
    V val = m_map.get(key);
    if (val == null) {
      K ceilingKey = m_map.ceilingKey(key);
      K floorKey = m_map.floorKey(key);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return m_map.get(floorKey);
      }
      if (floorKey == null) {
        return m_map.get(ceilingKey);
      }
      V floor = m_map.get(floorKey);
      V ceiling = m_map.get(ceilingKey);

      return m_interpolator.interpolate(
          floor, ceiling, m_inverseInterpolator.inverseInterpolate(floorKey, ceilingKey, key));
    } else {
      return val;
    }
  }

  /** Clears the contents. */
  public void clear() {
    m_map.clear();
  }
}