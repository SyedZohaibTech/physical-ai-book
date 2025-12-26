---
sidebar_position: 5
title: Sensor Simulation for Humanoid Robots
---

# Sensor Simulation for Humanoid Robots

Sensor simulation is critical for humanoid robotics, as these robots rely on diverse sensors to perceive their environment, maintain balance, and execute complex tasks. This chapter covers the simulation of various sensor types essential for humanoid robot operation.

## Overview of Humanoid Robot Sensors

Humanoid robots typically employ a variety of sensors:

### 1. Proprioceptive Sensors
- Joint position/velocity/effort sensors
- IMUs for orientation and acceleration
- Force/torque sensors at joints and feet

### 2. Exteroceptive Sensors
- Cameras (RGB, depth, stereo)
- LIDAR for 3D mapping
- Tactile sensors for manipulation

### 3. Environmental Sensors
- Microphones for audio input
- Temperature/humidity sensors
- Proximity sensors

## Camera Simulation

### 1. RGB Camera Simulation

Simulating RGB cameras in Gazebo:

```xml
<sensor name="camera" type="camera">
  <camera name="head_camera">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

In Unity, implementing camera simulation:

```csharp
using UnityEngine;

public class RGBCameraSimulator : MonoBehaviour
{
    [Header("Camera Settings")]
    public int width = 640;
    public int height = 480;
    public float fieldOfView = 60f;
    public float nearClip = 0.1f;
    public float farClip = 10.0f;
    
    [Header("Noise Settings")]
    public float noiseIntensity = 0.01f;
    public float gamma = 1.0f;
    
    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D outputTexture;
    
    void Start()
    {
        SetupCamera();
        CreateRenderTexture();
    }
    
    void SetupCamera()
    {
        cam = GetComponent<Camera>();
        cam.fieldOfView = fieldOfView;
        cam.nearClipPlane = nearClip;
        cam.farClipPlane = farClip;
    }
    
    void CreateRenderTexture()
    {
        renderTexture = new RenderTexture(width, height, 24);
        cam.targetTexture = renderTexture;
        outputTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
    }
    
    void Update()
    {
        CaptureAndProcessImage();
    }
    
    void CaptureAndProcessImage()
    {
        // Capture the camera image
        RenderTexture.active = renderTexture;
        outputTexture.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        outputTexture.Apply();
        
        // Apply noise simulation
        ApplyNoiseToImage(outputTexture);
        
        // Apply gamma correction
        ApplyGammaCorrection(outputTexture, gamma);
        
        RenderTexture.active = null;
    }
    
    void ApplyNoiseToImage(Texture2D texture)
    {
        Color[] pixels = texture.GetPixels();
        
        for (int i = 0; i < pixels.Length; i++)
        {
            // Add Gaussian noise
            float noise = Random.Range(-noiseIntensity, noiseIntensity);
            pixels[i].r = Mathf.Clamp01(pixels[i].r + noise);
            pixels[i].g = Mathf.Clamp01(pixels[i].g + noise);
            pixels[i].b = Mathf.Clamp01(pixels[i].b + noise);
        }
        
        texture.SetPixels(pixels);
        texture.Apply();
    }
    
    void ApplyGammaCorrection(Texture2D texture, float gamma)
    {
        Color[] pixels = texture.GetPixels();
        
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i] = new Color(
                Mathf.Pow(pixels[i].r, gamma),
                Mathf.Pow(pixels[i].g, gamma),
                Mathf.Pow(pixels[i].b, gamma)
            );
        }
        
        texture.SetPixels(pixels);
        texture.Apply();
    }
    
    // Method to get the processed image
    public Texture2D GetImage()
    {
        return outputTexture;
    }
}
```

### 2. Depth Camera Simulation

In Gazebo:

```xml
<sensor name="depth_camera" type="depth">
  <camera name="depth_head_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### 3. Stereo Camera Simulation

Stereo vision for depth perception:

```xml
<sensor name="stereo_camera" type="multicamera">
  <camera name="left_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <camera name="right_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <pose>0.1 0 0 0 0 0</pose>  <!-- Baseline between cameras -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

## LIDAR Simulation

### 1. 2D LIDAR

For navigation and obstacle detection:

```xml
<sensor name="laser_scan" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>  <!-- -90 degrees -->
        <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
```

### 2. 3D LIDAR

For full 3D mapping and localization:

```xml
<sensor name="3d_lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1080</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
        <max_angle>3.14159</max_angle>   <!-- 180 degrees -->
      </horizontal>
      <vertical>
        <samples>32</samples>
        <resolution>1</resolution>
        <min_angle>-0.5236</min_angle>  <!-- -30 degrees -->
        <max_angle>0.3491</max_angle>   <!-- 20 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
```

## IMU Simulation

IMUs are crucial for humanoid balance and orientation:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <topic>__default_topic__</topic>
  <visualize>false</visualize>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>  <!-- ~0.1 deg/s stddev -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- 0.017 m/s^2 stddev -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

Unity implementation for IMU simulation:

```csharp
using UnityEngine;

public class IMUSimulator : MonoBehaviour
{
    [Header("Noise Parameters")]
    public float gyroNoise = 0.0017f;  // rad/s
    public float accelNoise = 0.017f;  // m/s^2
    
    [Header("Bias Parameters")]
    public Vector3 gyroBias = Vector3.zero;
    public Vector3 accelBias = Vector3.zero;
    
    private float lastUpdate;
    private Vector3 lastAngularVelocity;
    private Vector3 lastLinearAcceleration;
    
    void Start()
    {
        lastUpdate = Time.time;
        lastAngularVelocity = Vector3.zero;
        lastLinearAcceleration = Physics.gravity;
    }
    
    void Update()
    {
        float deltaTime = Time.time - lastUpdate;
        lastUpdate = Time.time;
        
        // Calculate angular velocity (simplified)
        Vector3 angularVelocity = CalculateAngularVelocity();
        
        // Calculate linear acceleration (simplified)
        Vector3 linearAcceleration = CalculateLinearAcceleration();
        
        // Apply noise and bias
        Vector3 noisyGyro = AddNoiseToGyro(angularVelocity);
        Vector3 noisyAccel = AddNoiseToAccel(linearAcceleration);
        
        // Publish simulated IMU data
        PublishIMUData(noisyGyro, noisyAccel, deltaTime);
    }
    
    Vector3 CalculateAngularVelocity()
    {
        // In a real implementation, this would use physics engine data
        // For simulation, we'll estimate based on rotation changes
        return lastAngularVelocity;
    }
    
    Vector3 CalculateLinearAcceleration()
    {
        // Calculate linear acceleration from physics
        // This is a simplified approach
        Vector3 currentVelocity = GetComponent<Rigidbody>().velocity;
        Vector3 acceleration = (currentVelocity - lastLinearAcceleration) / Time.deltaTime;
        lastLinearAcceleration = currentVelocity;
        
        // Remove gravity component
        acceleration -= Physics.gravity;
        
        return acceleration;
    }
    
    Vector3 AddNoiseToGyro(Vector3 gyroReading)
    {
        Vector3 noise = new Vector3(
            Random.Range(-gyroNoise, gyroNoise),
            Random.Range(-gyroNoise, gyroNoise),
            Random.Range(-gyroNoise, gyroNoise)
        );
        
        return gyroReading + noise + gyroBias;
    }
    
    Vector3 AddNoiseToAccel(Vector3 accelReading)
    {
        Vector3 noise = new Vector3(
            Random.Range(-accelNoise, accelNoise),
            Random.Range(-accelNoise, accelNoise),
            Random.Range(-accelNoise, accelNoise)
        );
        
        return accelReading + noise + accelBias;
    }
    
    void PublishIMUData(Vector3 gyro, Vector3 accel, float deltaTime)
    {
        // In a real implementation, this would publish to ROS
        Debug.Log($"IMU: Gyro={gyro}, Accel={accel}");
    }
}
```

## Force/Torque Sensor Simulation

For humanoid robots, force/torque sensors at feet and hands are essential:

```xml
<sensor name="left_foot_force_torque" type="force_torque">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <force_torque>
    <frame>child</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
</sensor>
```

## Joint Position Sensors

Simulating encoder feedback:

```xml
<sensor name="joint_position_sensor" type="joint_position">
  <joint_name>left_knee_joint</joint_name>
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <topic>joint_position/left_knee</topic>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.001</stddev>  <!-- 1 mrad noise -->
  </noise>
</sensor>
```

## Sensor Fusion in Simulation

### 1. Kalman Filter Implementation

For sensor fusion in humanoid robots:

```csharp
using UnityEngine;
using System;

public class KalmanFilter
{
    private Matrix4x4 state;           // State vector [position, velocity]
    private Matrix4x4 covariance;      // Error covariance matrix
    private Matrix4x4 processNoise;    // Process noise covariance
    private Matrix4x4 measurementNoise; // Measurement noise covariance
    private Matrix4x4 transition;      // State transition matrix
    private Matrix4x4 observation;     // Observation matrix
    
    public KalmanFilter()
    {
        // Initialize with identity matrices
        state = Matrix4x4.zero;
        covariance = Matrix4x4.identity;
        processNoise = Matrix4x4.identity * 0.1f;
        measurementNoise = Matrix4x4.identity * 0.1f;
        transition = Matrix4x4.identity;
        observation = Matrix4x4.identity;
        
        // Set initial state and covariance
        covariance[0, 0] = 1.0f; // Position uncertainty
        covariance[1, 1] = 1.0f; // Velocity uncertainty
    }
    
    public void Predict(float deltaTime)
    {
        // Update transition matrix based on time step
        transition[0, 1] = deltaTime;
        
        // Predict state: x = F * x
        state = transition * state;
        
        // Predict covariance: P = F * P * F^T + Q
        Matrix4x4 Ft = Matrix4x4.transpose(transition);
        covariance = transition * covariance * Ft + processNoise;
    }
    
    public void Update(Vector2 measurement)
    {
        // Innovation: y = z - H * x
        Vector2 innovation = measurement - GetObservation(state);
        
        // Innovation covariance: S = H * P * H^T + R
        Matrix4x4 Ht = Matrix4x4.transpose(observation);
        Matrix4x4 innovationCov = observation * covariance * Ht + measurementNoise;
        
        // Kalman gain: K = P * H^T * S^(-1)
        Matrix4x4 kalmanGain = covariance * Ht * Matrix4x4.inverse(innovationCov);
        
        // Update state: x = x + K * y
        state = state + (Matrix4x4)kalmanGain * (Matrix4x4)innovation;
        
        // Update covariance: P = (I - K * H) * P
        Matrix4x4 I = Matrix4x4.identity;
        covariance = (I - kalmanGain * observation) * covariance;
    }
    
    private Vector2 GetObservation(Matrix4x4 stateMatrix)
    {
        return new Vector2(stateMatrix[0, 0], stateMatrix[1, 0]);
    }
    
    public Vector2 GetState()
    {
        return new Vector2(state[0, 0], state[1, 0]);
    }
}
```

### 2. Extended Kalman Filter for Nonlinear Systems

For more complex sensor fusion:

```csharp
using UnityEngine;
using System;

public class ExtendedKalmanFilter
{
    private Matrix4x4 state;
    private Matrix4x4 covariance;
    private Matrix4x4 processNoise;
    private Matrix4x4 measurementNoise;
    
    public ExtendedKalmanFilter(int stateSize)
    {
        state = new Matrix4x4();
        covariance = Matrix4x4.identity;
        processNoise = Matrix4x4.identity * 0.1f;
        measurementNoise = Matrix4x4.identity * 0.1f;
    }
    
    public void Predict(System.Func<Matrix4x4, Matrix4x4> stateTransitionFunc,
                        System.Func<Matrix4x4, Matrix4x4> jacobianFunc,
                        float deltaTime)
    {
        // Nonlinear state prediction
        Matrix4x4 predictedState = stateTransitionFunc(state);
        
        // Jacobian of state transition function
        Matrix4x4 jacobian = jacobianFunc(state);
        Matrix4x4 jacobianT = Matrix4x4.transpose(jacobian);
        
        // Predict covariance
        covariance = jacobian * covariance * jacobianT + processNoise;
        
        // Update state
        state = predictedState;
    }
    
    public void Update(Vector2 measurement,
                       System.Func<Matrix4x4, Vector2> observationFunc,
                       System.Func<Matrix4x4, Matrix4x4> observationJacobianFunc)
    {
        // Predicted measurement
        Vector2 predictedMeasurement = observationFunc(state);
        
        // Innovation
        Vector2 innovation = measurement - predictedMeasurement;
        
        // Observation Jacobian
        Matrix4x4 H = observationJacobianFunc(state);
        Matrix4x4 Ht = Matrix4x4.transpose(H);
        
        // Innovation covariance
        Matrix4x4 innovationCov = H * covariance * Ht + measurementNoise;
        
        // Kalman gain
        Matrix4x4 kalmanGain = covariance * Ht * Matrix4x4.inverse(innovationCov);
        
        // Update state
        state = state + (Matrix4x4)kalmanGain * (Matrix4x4)innovation;
        
        // Update covariance
        Matrix4x4 I = Matrix4x4.identity;
        covariance = (I - kalmanGain * H) * covariance;
    }
}
```

## Sensor Calibration and Validation

### 1. Calibration Procedures

For accurate sensor simulation:

```csharp
using UnityEngine;
using System.Collections;

public class SensorCalibrator : MonoBehaviour
{
    public IMUSimulator imuSim;
    public RGBCameraSimulator cameraSim;
    
    [Header("Calibration Settings")]
    public int calibrationSamples = 1000;
    public float calibrationDuration = 10.0f;
    
    private bool isCalibrating = false;
    private int sampleCount = 0;
    
    public void StartCalibration()
    {
        if (!isCalibrating)
        {
            StartCoroutine(RunCalibration());
        }
    }
    
    IEnumerator RunCalibration()
    {
        isCalibrating = true;
        sampleCount = 0;
        
        // Collect samples over time
        float sampleInterval = calibrationDuration / calibrationSamples;
        
        while (sampleCount < calibrationSamples)
        {
            // Collect sensor data
            CollectCalibrationSample();
            
            sampleCount++;
            yield return new WaitForSeconds(sampleInterval);
        }
        
        // Process collected data and update calibration
        ProcessCalibrationData();
        
        isCalibrating = false;
        Debug.Log("Calibration complete!");
    }
    
    void CollectCalibrationSample()
    {
        // Collect IMU data for bias estimation
        // Collect camera data for intrinsic/extrinsic calibration
    }
    
    void ProcessCalibrationData()
    {
        // Process collected samples to estimate biases and parameters
        // Update sensor models with calibrated values
    }
}
```

### 2. Validation Against Physical Sensors

To ensure simulation accuracy:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorValidator : MonoBehaviour
{
    [Header("Validation Settings")]
    public float tolerance = 0.1f;
    
    private List<float> simValues = new List<float>();
    private List<float> realValues = new List<float>();
    
    public void AddSimulationValue(float value)
    {
        simValues.Add(value);
    }
    
    public void AddRealValue(float value)
    {
        realValues.Add(value);
    }
    
    public float CalculateRMSE()
    {
        if (simValues.Count != realValues.Count || simValues.Count == 0)
            return float.PositiveInfinity;
        
        float sumSquaredErrors = 0.0f;
        for (int i = 0; i < simValues.Count; i++)
        {
            float error = simValues[i] - realValues[i];
            sumSquaredErrors += error * error;
        }
        
        return Mathf.Sqrt(sumSquaredErrors / simValues.Count);
    }
    
    public bool IsWithinTolerance()
    {
        return CalculateRMSE() <= tolerance;
    }
}
```

## Advanced Sensor Simulation Techniques

### 1. Domain Randomization

To improve sim-to-real transfer:

```csharp
using UnityEngine;

public class DomainRandomization : MonoBehaviour
{
    [Header("Lighting Randomization")]
    public bool randomizeLighting = true;
    public Color minLightColor = Color.white;
    public Color maxLightColor = Color.white;
    
    [Header("Material Randomization")]
    public bool randomizeMaterials = true;
    public float minRoughness = 0.1f;
    public float maxRoughness = 0.9f;
    
    [Header("Sensor Noise Randomization")]
    public bool randomizeSensorNoise = true;
    public float minNoise = 0.001f;
    public float maxNoise = 0.01f;
    
    void Start()
    {
        if (randomizeLighting)
            RandomizeLighting();
        
        if (randomizeMaterials)
            RandomizeMaterials();
        
        if (randomizeSensorNoise)
            RandomizeSensorNoise();
    }
    
    void RandomizeLighting()
    {
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            light.color = new Color(
                Random.Range(minLightColor.r, maxLightColor.r),
                Random.Range(minLightColor.g, maxLightColor.g),
                Random.Range(minLightColor.b, maxLightColor.b)
            );
            
            light.intensity = Random.Range(0.5f, 1.5f);
        }
    }
    
    void RandomizeMaterials()
    {
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            Material material = renderer.material;
            if (material.HasProperty("_Roughness"))
            {
                material.SetFloat("_Roughness", 
                    Random.Range(minRoughness, maxRoughness));
            }
        }
    }
    
    void RandomizeSensorNoise()
    {
        // Randomize noise parameters for sensors
        // This would typically be implemented in each sensor class
    }
}
```

### 2. Sensor Failure Simulation

For robustness testing:

```csharp
using UnityEngine;

public class SensorFailureSimulator : MonoBehaviour
{
    [Header("Failure Parameters")]
    public float failureProbability = 0.001f;  // Per frame
    public float recoveryProbability = 0.1f;   // Per frame when failed
    public float driftRate = 0.001f;           // Per frame when drifting
    
    private bool isFailed = false;
    private bool isDrifting = false;
    private float driftOffset = 0.0f;
    
    void Update()
    {
        if (!isFailed)
        {
            // Check for failure
            if (Random.value < failureProbability)
            {
                isFailed = true;
                Debug.Log("Sensor failure simulated!");
            }
        }
        else
        {
            // Check for recovery
            if (Random.value < recoveryProbability)
            {
                isFailed = false;
                isDrifting = false;
                driftOffset = 0.0f;
                Debug.Log("Sensor recovered!");
            }
            else if (Random.value < 0.01f) // Low probability of drift
            {
                isDrifting = true;
            }
        }
        
        if (isDrifting)
        {
            driftOffset += Random.Range(-driftRate, driftRate);
        }
    }
    
    public float ApplyFailureToValue(float value)
    {
        if (isFailed)
        {
            // Return a failure value (e.g., zero or NaN)
            return 0.0f;  // Or float.NaN for more realistic failure
        }
        
        // Apply drift if applicable
        return value + driftOffset;
    }
}
```

Accurate sensor simulation is fundamental to developing robust humanoid robots. By properly modeling sensor characteristics, noise, and failure modes, developers can create more reliable and transferable robot behaviors that will perform well in real-world scenarios.