# Chapter 3: High-Fidelity Rendering & Human-Robot Interaction in Unity

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand Unity's rendering capabilities for high-fidelity visualization
- Implement Unity XR Interaction Framework for human-robot interaction
- Configure Unity assets for humanoid robot models
- Create HRI scenario examples with realistic interaction patterns
- Follow best practices for Unity-ROS integration
- Build immersive experiences for robot teleoperation and monitoring

## Introduction to Unity Rendering Capabilities

Unity is a powerful real-time 3D development platform that excels at creating high-fidelity visual experiences. For robotics applications, Unity provides photorealistic rendering capabilities that can complement physics simulation from tools like Gazebo, creating a complete digital twin experience.

Unity's rendering pipeline includes advanced features such as:
- High Definition Render Pipeline (HDRP) for photorealistic visuals
- Universal Render Pipeline (URP) for performance across platforms
- Real-time lighting and global illumination
- Advanced materials and shaders
- Post-processing effects for enhanced visual quality

### Unity vs. Gazebo for Visualization

While Gazebo excels at accurate physics simulation, Unity focuses on high-fidelity rendering:

- **Gazebo**: Accurate physics, collision detection, sensor simulation
- **Unity**: Photorealistic rendering, immersive visualization, human-robot interaction
- **Integration**: Use Gazebo for physics, Unity for visualization (GzUnity bridge)

## Unity XR Interaction Framework

### Overview of XR Interaction Framework

The Unity XR Interaction Framework provides a comprehensive set of tools for creating human-robot interaction scenarios. It enables developers to create immersive experiences where humans can interact with robots in realistic ways.

Key components of the XR Interaction Framework include:
- **Interactors**: Objects that can initiate interactions (controllers, hands, etc.)
- **Interactables**: Objects that can be interacted with
- **Interaction Groups**: Collections of interactors that work together
- **Interaction Affordance**: Visual feedback for interaction states

### Setting Up XR Interaction Framework

To use the XR Interaction Framework in your Unity project:

1. **Install the package** via Unity Package Manager:
   - Window → Package Manager
   - Select "XR Interaction Framework" from the list
   - Install the latest stable version

2. **Add XR Interaction Manager** to your scene:
   ```csharp
   // Add this component to your main camera or a dedicated manager object
   using UnityEngine.XR.Interaction.Toolkit;

   public class HRIManager : MonoBehaviour
   {
       [SerializeField] private XRInteractionManager interactionManager;
   }
   ```

### Basic HRI Components

```csharp
// Example: Interactive robot control panel
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

public class RobotControlPanel : MonoBehaviour
{
    [SerializeField] private XRBaseInteractable controlButton;

    private void Start()
    {
        // Subscribe to interaction events
        controlButton.selectEntered.AddListener(OnButtonPressed);
    }

    private void OnButtonPressed(SelectEnterEventArgs args)
    {
        // Handle robot control command
        Debug.Log("Robot control command issued");
    }
}
```

### Advanced HRI Patterns

For humanoid robots, consider these interaction patterns:

1. **Teleoperation**: Direct control of robot movements
2. **Supervisory Control**: High-level command giving
3. **Monitoring**: Observation and data analysis
4. **Collaboration**: Human-robot teamwork scenarios

## Unity Asset Configuration for Humanoid Robots

### Importing Robot Models

When importing humanoid robot models into Unity:

1. **Model Format**: Prefer FBX format for best compatibility
2. **Scale**: Ensure models are correctly scaled (typically 1:1 with real robot)
3. **Coordinate System**: Unity uses left-handed coordinate system (Z-forward)
4. **Rig Configuration**: Set up humanoid rig for animation system

### Robot Model Setup

```csharp
// Robot model configuration script
using UnityEngine;

public class RobotModelConfiguration : MonoBehaviour
{
    [Header("Robot Dimensions")]
    public float robotHeight = 1.5f;  // meters
    public float robotWidth = 0.5f;   // meters
    public float robotDepth = 0.5f;   // meters

    [Header("Joint Configuration")]
    public Transform[] jointTransforms;
    public string[] jointNames;

    [Header("Visual Materials")]
    public Material robotBodyMaterial;
    public Material jointMaterial;

    void Start()
    {
        ConfigureRobotMaterials();
        ValidateJointSetup();
    }

    void ConfigureRobotMaterials()
    {
        // Apply materials to robot components
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            if (renderer.name.Contains("joint"))
                renderer.material = jointMaterial;
            else
                renderer.material = robotBodyMaterial;
        }
    }

    void ValidateJointSetup()
    {
        // Verify all joints are properly configured
        foreach (Transform joint in jointTransforms)
        {
            if (joint == null)
                Debug.LogWarning($"Missing joint in configuration");
        }
    }
}
```

### Animation and Locomotion Setup

For humanoid robots, configure the Animation Rigging package:

```csharp
// Animation setup for humanoid locomotion
using UnityEngine;
using UnityEngine.Animations.Rigging;

public class HumanoidLocomotionSetup : MonoBehaviour
{
    [Header("Rigging Setup")]
    public MultiAimConstraint headAimConstraint;
    public MultiAimConstraint torsoAimConstraint;

    [Header("Locomotion Targets")]
    public Transform lookAtTarget;
    public Transform bodyOrientationTarget;

    void Start()
    {
        SetupLocomotionRig();
    }

    void SetupLocomotionRig()
    {
        if (headAimConstraint != null && lookAtTarget != null)
        {
            headAimConstraint.data.target = lookAtTarget;
        }

        if (torsoAimConstraint != null && bodyOrientationTarget != null)
        {
            torsoAimConstraint.data.target = bodyOrientationTarget;
        }
    }
}
```

## Implementing HRI Scenario Examples

### Scenario 1: Robot Teleoperation Interface

Creating an immersive teleoperation experience:

```csharp
// Teleoperation interface script
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

public class RobotTeleoperationInterface : MonoBehaviour
{
    [Header("Teleoperation Controls")]
    public XRBaseController leftController;
    public XRBaseController rightController;

    [Header("Robot References")]
    public GameObject robotModel;
    public Transform robotRoot;

    [Header("Visualization")]
    public LineRenderer controlRay;
    public GameObject interactionPoint;

    private void Update()
    {
        HandleTeleoperationInput();
        UpdateVisualization();
    }

    private void HandleTeleoperationInput()
    {
        // Map controller inputs to robot commands
        if (leftController.activateButton.isPressed)
        {
            // Handle left controller input
            Vector2 leftStick = leftController.primary2DAxis.ReadValue();
            SendRobotMovementCommand(leftStick.x, leftStick.y);
        }

        if (rightController.activateButton.isPressed)
        {
            // Handle right controller input
            Vector2 rightStick = rightController.primary2DAxis.ReadValue();
            SendRobotLookCommand(rightStick.x, rightStick.y);
        }
    }

    private void SendRobotMovementCommand(float x, float y)
    {
        // Send movement command to robot (via ROS bridge)
        Debug.Log($"Sending movement command: ({x}, {y})");
    }

    private void SendRobotLookCommand(float x, float y)
    {
        // Send look command to robot
        Debug.Log($"Sending look command: ({x}, {y})");
    }

    private void UpdateVisualization()
    {
        // Update visual feedback for teleoperation
        if (controlRay != null)
        {
            controlRay.SetPosition(0, leftController.transform.position);
            controlRay.SetPosition(1, leftController.transform.position +
                                  leftController.transform.forward * 5f);
        }
    }
}
```

### Scenario 2: Robot Monitoring Dashboard

Creating a monitoring interface for robot status:

```csharp
// Robot monitoring dashboard
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class RobotMonitoringDashboard : MonoBehaviour
{
    [Header("UI Elements")]
    public Text robotStatusText;
    public Text batteryLevelText;
    public Text sensorStatusText;
    public Image batteryBar;

    [Header("Robot Data")]
    public float maxBattery = 100f;
    private float currentBattery = 100f;

    [Header("Sensor Data")]
    public List<string> sensorNames;
    public List<bool> sensorStatuses;

    void Start()
    {
        InitializeDashboard();
    }

    void Update()
    {
        UpdateDashboard();
    }

    void InitializeDashboard()
    {
        // Initialize dashboard with default values
        currentBattery = maxBattery;
        UpdateBatteryDisplay();
        UpdateSensorStatus();
    }

    void UpdateDashboard()
    {
        // Simulate battery drain
        currentBattery = Mathf.Max(0, currentBattery - Time.deltaTime * 0.1f);
        UpdateBatteryDisplay();

        // Randomly toggle sensor statuses for demo
        for (int i = 0; i < sensorStatuses.Count; i++)
        {
            sensorStatuses[i] = Random.value > 0.1f; // 90% operational
        }
        UpdateSensorStatus();
    }

    void UpdateBatteryDisplay()
    {
        float batteryPercent = currentBattery / maxBattery;
        batteryBar.fillAmount = batteryPercent;
        batteryLevelText.text = $"Battery: {(int)(batteryPercent * 100)}%";

        // Change color based on battery level
        if (batteryPercent < 0.2f)
            batteryBar.color = Color.red;
        else if (batteryPercent < 0.5f)
            batteryBar.color = Color.yellow;
        else
            batteryBar.color = Color.green;
    }

    void UpdateSensorStatus()
    {
        string status = "";
        for (int i = 0; i < sensorNames.Count; i++)
        {
            status += $"{sensorNames[i]}: {(sensorStatuses[i] ? "OK" : "ERROR")}\n";
        }
        sensorStatusText.text = status;
    }
}
```

### Scenario 3: Collaborative Task Interface

Creating interfaces for human-robot collaboration:

```csharp
// Collaborative task interface
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using System.Collections.Generic;

public class CollaborativeTaskInterface : MonoBehaviour
{
    [Header("Collaboration Objects")]
    public List<GameObject> taskObjects;
    public Transform[] workstations;

    [Header("Interaction Setup")]
    public XRSocketInteractor humanSocket;
    public XRSocketInteractor robotSocket;

    [Header("Task Progress")]
    public Slider taskProgress;
    public Text taskStatusText;

    private int completedTasks = 0;
    private int totalTasks = 5;

    void Start()
    {
        InitializeCollaborationInterface();
    }

    void InitializeCollaborationInterface()
    {
        // Setup sockets for object interaction
        humanSocket.enabled = true;
        robotSocket.enabled = true;

        // Initialize task objects
        foreach (GameObject obj in taskObjects)
        {
            obj.layer = LayerMask.NameToLayer("TaskObject");
            foreach (Transform child in obj.GetComponentsInChildren<Transform>())
            {
                child.gameObject.layer = LayerMask.NameToLayer("TaskObject");
            }
        }
    }

    public void OnObjectPlaced(GameObject obj, bool byHuman)
    {
        completedTasks++;
        UpdateTaskProgress();

        string actor = byHuman ? "Human" : "Robot";
        taskStatusText.text = $"{actor} placed object. Tasks completed: {completedTasks}/{totalTasks}";

        if (completedTasks >= totalTasks)
        {
            CompleteTask();
        }
    }

    void UpdateTaskProgress()
    {
        float progress = (float)completedTasks / totalTasks;
        taskProgress.value = progress;
    }

    void CompleteTask()
    {
        taskStatusText.text = "Collaborative task completed successfully!";
        // Add celebration effects or next phase activation
    }
}
```

## Unity Rendering Workflow Example

### Complete Rendering Pipeline Setup

```csharp
// Complete rendering pipeline setup for robot visualization
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class RobotRenderingPipeline : MonoBehaviour
{
    [Header("Lighting Setup")]
    public Light mainLight;
    public Light[] additionalLights;

    [Header("Post-Processing")]
    public Volume postProcessingVolume;
    public DepthOfField depthOfField;

    [Header("Robot Visualization")]
    public GameObject robotModel;
    public Material[] robotMaterials;

    [Header("Camera Setup")]
    public Camera mainCamera;
    public float followDistance = 5f;
    public float followHeight = 2f;

    void Start()
    {
        SetupRenderingPipeline();
        ConfigureRobotMaterials();
        SetupCameraFollow();
    }

    void SetupRenderingPipeline()
    {
        // Configure HDRP settings for photorealistic rendering
        if (RenderPipelineManager.currentPipeline is HDRenderPipeline)
        {
            ConfigureHDRPSettings();
        }

        // Setup lighting
        ConfigureLighting();
    }

    void ConfigureHDRPSettings()
    {
        // Configure advanced rendering features
        // This would include settings for shadows, reflections, etc.
    }

    void ConfigureLighting()
    {
        if (mainLight != null)
        {
            mainLight.type = LightType.Directional;
            mainLight.intensity = 3.14f; // Physical light intensity
            mainLight.shadows = LightShadows.Soft;
        }
    }

    void ConfigureRobotMaterials()
    {
        // Apply physically-based materials to robot
        foreach (Material mat in robotMaterials)
        {
            if (mat.HasProperty("_Smoothness"))
            {
                mat.SetFloat("_Smoothness", 0.5f);
            }
            if (mat.HasProperty("_Metallic"))
            {
                mat.SetFloat("_Metallic", 0.8f);
            }
        }
    }

    void SetupCameraFollow()
    {
        // Implement smooth camera following for robot
        StartCoroutine(FollowRobotSmoothly());
    }

    System.Collections.IEnumerator FollowRobotSmoothly()
    {
        Vector3 offset = new Vector3(0, followHeight, -followDistance);

        while (true)
        {
            Vector3 targetPosition = robotModel.transform.position +
                                   robotModel.transform.TransformDirection(offset);
            mainCamera.transform.position = Vector3.Lerp(
                mainCamera.transform.position, targetPosition, Time.deltaTime * 2f);

            mainCamera.transform.LookAt(robotModel.transform);

            yield return null;
        }
    }
}
```

## Best Practices for Unity-ROS Integration

### ROS-TCP-Connector Setup

The Unity ROS TCP Connector is essential for communication between Unity and ROS:

```csharp
// ROS connection manager
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ROSConnectionManager : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Robot Topics")]
    public string jointStatesTopic = "/joint_states";
    public string cmdVelTopic = "/cmd_vel";

    private ROSConnection rosConnection;

    void Start()
    {
        ConnectToROS();
    }

    void ConnectToROS()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.RegisterPublisher<Unity.Robotics.ROS_TCPConnector.MessageTypes.Std_msgs.StringMsg>(cmdVelTopic);

        // Start listening for robot data
        rosConnection.Subscribe<Unity.Robotics.ROS_TCPConnector.MessageTypes.Sensor_msgs.JointStateMsg>(
            jointStatesTopic, OnJointStateReceived);
    }

    void OnJointStateReceived(Unity.Robotics.ROS_TCPConnector.MessageTypes.Sensor_msgs.JointStateMsg jointState)
    {
        // Update robot model based on received joint states
        UpdateRobotVisualization(jointState);
    }

    void UpdateRobotVisualization(Unity.Robotics.ROS_TCPConnector.MessageTypes.Sensor_msgs.JointStateMsg jointState)
    {
        // Update robot model joints based on ROS data
        // Implementation depends on robot model structure
    }
}
```

### Performance Optimization

1. **LOD (Level of Detail)**: Use simpler models when far from camera
2. **Occlusion Culling**: Hide objects not visible to the camera
3. **Texture Streaming**: Load textures as needed
4. **Object Pooling**: Reuse objects instead of creating/destroying

### Data Synchronization

Ensure Unity visualization stays synchronized with ROS simulation:

```csharp
// Synchronization manager
using UnityEngine;
using System.Collections;

public class VisualizationSynchronizer : MonoBehaviour
{
    [Header("Synchronization")]
    public float syncInterval = 0.033f; // ~30 FPS
    public float maxSyncDelay = 0.1f;

    private Coroutine syncCoroutine;

    void Start()
    {
        StartSynchronization();
    }

    void StartSynchronization()
    {
        syncCoroutine = StartCoroutine(SynchronizeWithROS());
    }

    IEnumerator SynchronizeWithROS()
    {
        while (true)
        {
            // Request latest robot state from ROS
            RequestRobotState();

            // Wait for specified interval
            yield return new WaitForSeconds(syncInterval);
        }
    }

    void RequestRobotState()
    {
        // Implementation to request current robot state from ROS
        // This would typically involve sending a service call or topic request
    }
}
```

## Troubleshooting Common HRI Issues

### Performance Problems
- Reduce polygon count of robot models
- Use occlusion culling for complex environments
- Limit post-processing effects
- Use lower-resolution textures

### Interaction Problems
- Verify XR Interaction Framework components are properly configured
- Check layer masks for interaction
- Ensure proper scale of interactive objects
- Validate controller input mappings

### Synchronization Issues
- Check network connectivity between Unity and ROS
- Verify correct topic names and message types
- Monitor for dropped messages
- Adjust synchronization frequency as needed

## Summary

High-fidelity rendering and human-robot interaction in Unity provides an immersive visualization layer that complements the accurate physics simulation from Gazebo. By implementing the Unity XR Interaction Framework and following best practices for Unity-ROS integration, you can create compelling HRI experiences that enhance robot development and testing.

The key to successful HRI implementation is balancing visual fidelity with performance while ensuring reliable synchronization between the simulation and visualization systems. As you develop more complex HRI scenarios, focus on intuitive interaction patterns that enhance human understanding and control of robotic systems.

## References

Ferreira, A., et al. (2020). Simulation tools for robotics: Comparison of Gazebo and Webots. *IEEE Latin America Transactions*, 18(4), 623-630.

Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2350-2354.

Maggio, M., et al. (2017). Gazebo as a tool for software development in robotics: The case of fault-tolerance. *Annual IEEE International Systems Conference*, 1-7.

Unity Technologies. (2021). Best practices for sim-to-real transfer in robotics. *Unity Technical Report*.

O'Flaherty, R., et al. (2019). The Open-Source ROS Package for Simultaneous Localization and Mapping. *Journal of Software Engineering in Robotics*, 10(1), 45-58.