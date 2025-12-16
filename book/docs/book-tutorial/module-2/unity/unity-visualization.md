---
sidebar_position: 3
title: "Unity Visualization for Digital Twins"
---

# Unity Visualization for Digital Twins

Unity provides high-fidelity visualization capabilities that complement Gazebo's physics simulation. This chapter covers setting up Unity for digital twin visualization and best practices for creating immersive environments.

## Introduction to Unity for Digital Twins

Unity serves as the visualization layer in digital twin applications, providing photorealistic rendering, advanced lighting, and interactive capabilities. While Gazebo excels at physics simulation, Unity offers superior graphics quality and user interaction possibilities.

## Setting Up Unity for Digital Twin Applications

### Prerequisites

- Unity Hub (recommended)
- Unity Editor 2021.3 LTS or later
- Visual Studio or Rider for scripting
- Git LFS for asset versioning

### Recommended Project Structure

```
UnityDigitalTwin/
├── Assets/
│   ├── Scenes/           # Scene files
│   ├── Scripts/          # C# scripts
│   ├── Materials/        # Material definitions
│   ├── Models/           # 3D models
│   ├── Textures/         # Texture assets
│   ├── Prefabs/          # Reusable objects
│   └── Plugins/          # Third-party plugins
├── Packages/             # Package Manager packages
└── ProjectSettings/      # Project configurations
```

## Environment Setup

### Creating a New Digital Twin Project

1. Open Unity Hub and create a new 3D Core project
2. Set up the following recommended settings:
   - Render Pipeline: Universal Render Pipeline (URP) for performance
   - Quality Settings: Balanced for real-time simulation

### Configuring URP Pipeline

```csharp
// Example URP Asset Configuration
using UnityEngine;
using UnityEngine.Rendering.Universal;

public class DigitalTwinURPSetup : MonoBehaviour
{
    [Header("Quality Settings")]
    public int shadowResolution = 1024;
    public float shadowDistance = 50f;
    public int antiAliasing = 2; // MSAA

    void Start()
    {
        // Configure rendering settings for digital twin
        var universalRenderPipelineAsset = GraphicsSettings.renderPipelineAsset as UniversalRenderPipelineAsset;
        if (universalRenderPipelineAsset != null)
        {
            universalRenderPipelineAsset.shadowDistance = shadowDistance;
            universalRenderPipelineAsset.shadowCascadeCount = 2;
        }
    }
}
```

## Scene Architecture

### Digital Twin Scene Components

A typical digital twin scene consists of:

1. **Environment**: Physical space representation
2. **Assets**: Robots, equipment, and dynamic elements
3. **Cameras**: Multiple viewpoints for monitoring
4. **Lighting**: Realistic illumination
5. **UI Elements**: Status displays and controls

### Environment Creation

```csharp
using UnityEngine;

public class EnvironmentBuilder : MonoBehaviour
{
    [Header("Terrain Settings")]
    public float terrainWidth = 100f;
    public float terrainHeight = 100f;
    public AnimationCurve heightMap = AnimationCurve.Linear(0, 0, 1, 1);

    [Header("Building Parameters")]
    public Vector3 buildingSize = new Vector3(10f, 5f, 15f);
    public Transform buildingParent;

    void Start()
    {
        CreateTerrain();
        CreateBuildings();
    }

    void CreateTerrain()
    {
        // Create terrain programmatically
        Terrain terrain = Terrain.CreateEcosystemTerrain();
        terrain.terrainData.size = new Vector3(terrainWidth, 10f, terrainHeight);

        // Apply heightmap
        float[,] heights = new float[terrain.terrainData.heightmapResolution,
                                   terrain.terrainData.heightmapResolution];

        for (int x = 0; x < terrain.terrainData.heightmapResolution; x++)
        {
            for (int z = 0; z < terrain.terrainData.heightmapResolution; z++)
            {
                float xPercent = (float)x / (terrain.terrainData.heightmapResolution - 1);
                float zPercent = (float)z / (terrain.terrainData.heightmapResolution - 1);

                heights[x, z] = heightMap.Evaluate(xPercent) * heightMap.Evaluate(zPercent);
            }
        }

        terrain.terrainData.SetHeights(0, 0, heights);
    }

    void CreateBuildings()
    {
        // Instantiate building prefabs
        for (int i = 0; i < 5; i++)
        {
            GameObject building = GameObject.CreatePrimitive(PrimitiveType.Cube);
            building.transform.position = new Vector3(
                Random.Range(-terrainWidth/2, terrainWidth/2),
                buildingSize.y/2,
                Random.Range(-terrainHeight/2, terrainHeight/2)
            );
            building.transform.localScale = buildingSize;
            building.transform.parent = buildingParent;

            // Apply building material
            building.GetComponent<Renderer>().material = Resources.Load<Material>("BuildingMat");
        }
    }
}
```

## Robot and Asset Integration

### Importing Robot Models

Unity supports various 3D model formats (FBX, OBJ, glTF). For digital twins, consider:

1. **LOD Systems**: Multiple detail levels for performance
2. **Skinned Meshes**: For articulated robots
3. **Optimized Geometry**: Reduced polygon counts for real-time performance

```csharp
using UnityEngine;

public class RobotImporter : MonoBehaviour
{
    [Header("Robot Configuration")]
    public GameObject robotPrefab;
    public Transform spawnPoint;
    public bool useLOD = true;

    [Header("Animation Rigging")]
    public bool enableRigging = true;

    void Start()
    {
        SpawnRobot();
    }

    void SpawnRobot()
    {
        GameObject robotInstance = Instantiate(robotPrefab, spawnPoint.position, spawnPoint.rotation);

        if (useLOD)
        {
            SetupLODGroup(robotInstance);
        }

        if (enableRigging)
        {
            SetupAnimationRigging(robotInstance);
        }
    }

    void SetupLODGroup(GameObject robot)
    {
        LODGroup lodGroup = robot.AddComponent<LODGroup>();

        Renderer[] renderers = robot.GetComponentsInChildren<Renderer>();

        LOD[] lods = new LOD[3];

        // LOD 0: High detail (distance 0-20m)
        lods[0] = new LOD(0.5f, renderers);

        // LOD 1: Medium detail (distance 20-50m)
        Renderer[] mediumLODRenderers = GetMediumLODRenderers(renderers);
        lods[1] = new LOD(0.25f, mediumLODRenderers);

        // LOD 2: Low detail (distance 50m+)
        Renderer[] lowLODRenderers = GetLowLODRenderers(renderers);
        lods[2] = new LOD(0.1f, lowLODRenderers);

        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }

    Renderer[] GetMediumLODRenderers(Renderer[] original)
    {
        // Implementation for medium detail renderers
        return original;
    }

    Renderer[] GetLowLODRenderers(Renderer[] original)
    {
        // Implementation for low detail renderers
        return new Renderer[0];
    }

    void SetupAnimationRigging(GameObject robot)
    {
        // Setup animation rigging for articulated parts
        // Requires Animation Rigging package
    }
}
```

## Real-time Data Integration

### Connecting to Simulation Data

Unity can receive real-time data from Gazebo through various protocols:

1. **ROS Bridge**: Using rosbridge_suite
2. **Custom TCP/UDP**: Direct socket communication
3. **REST APIs**: For simpler data exchange

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;

public class DataReceiver : MonoBehaviour
{
    [Header("Connection Settings")]
    public string ipAddress = "127.0.0.1";
    public int port = 5000;

    [Header("Robot Data")]
    public Transform robotTransform;
    public List<Transform> jointTransforms = new List<Transform>();

    private TcpClient tcpClient;
    private NetworkStream stream;

    void Start()
    {
        ConnectToSimulation();
    }

    void ConnectToSimulation()
    {
        try
        {
            tcpClient = new TcpClient(ipAddress, port);
            stream = tcpClient.GetStream();

            StartCoroutine(ReceiveData());
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to connect: {e.Message}");
        }
    }

    IEnumerator ReceiveData()
    {
        byte[] buffer = new byte[1024];

        while (tcpClient.Connected)
        {
            yield return new WaitForSeconds(0.01f); // ~100Hz update

            if (stream.DataAvailable)
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                string data = Encoding.UTF8.GetString(buffer, 0, bytesRead);

                ProcessSimulationData(data);
            }
        }
    }

    void ProcessSimulationData(string jsonData)
    {
        // Parse JSON data from Gazebo
        RobotState state = JsonUtility.FromJson<RobotState>(jsonData);

        // Update robot position and orientation
        robotTransform.position = new Vector3(state.x, state.y, state.z);
        robotTransform.rotation = Quaternion.Euler(state.roll, state.pitch, state.yaw);

        // Update joint positions
        for (int i = 0; i < jointTransforms.Count && i < state.jointPositions.Count; i++)
        {
            jointTransforms[i].localRotation = Quaternion.Euler(
                0, 0, state.jointPositions[i] * Mathf.Rad2Deg
            );
        }
    }

    [System.Serializable]
    public class RobotState
    {
        public float x, y, z;
        public float roll, pitch, yaw;
        public List<float> jointPositions = new List<float>();
    }

    void OnDestroy()
    {
        if (tcpClient != null)
        {
            tcpClient.Close();
        }
    }
}
```

## Advanced Visualization Techniques

### Shader Programming for Digital Twins

Custom shaders can enhance the visualization of digital twin data:

```hlsl
// RobotHighlight.shader
Shader "DigitalTwin/RobotHighlight"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _Color ("Color", Color) = (1,1,1,1)
        _HighlightColor ("Highlight Color", Color) = (0.2, 0.8, 1, 1)
        _HighlightPower ("Highlight Power", Range(0, 1)) = 0.5
        _EdgeThickness ("Edge Thickness", Range(0, 1)) = 0.5
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                float3 normal : NORMAL;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float3 worldNormal : TEXCOORD1;
                float3 worldPos : TEXCOORD2;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            float4 _Color;
            float4 _HighlightColor;
            float _HighlightPower;
            float _EdgeThickness;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                o.worldNormal = mul((float3x3)unity_WorldToObject, v.normal);
                o.worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.uv) * _Color;

                // Calculate rim lighting for highlighting
                float3 viewDir = normalize(_WorldSpaceCameraPos - i.worldPos);
                float rim = 1.0 - saturate(dot(normalize(i.worldNormal), viewDir));
                rim = pow(rim, 1.0 / _EdgeThickness);

                // Blend with highlight color
                col.rgb = lerp(col.rgb, _HighlightColor.rgb, rim * _HighlightPower);

                return col;
            }
            ENDCG
        }
    }
}
```

### Particle Systems for Sensor Visualization

Visualize sensor data using particle systems:

```csharp
using UnityEngine;

public class SensorVisualization : MonoBehaviour
{
    [Header("LiDAR Visualization")]
    public ParticleSystem lidarParticleSystem;
    public Transform robotBase;

    [Header("Camera Feed")]
    public RenderTexture cameraFeedTexture;
    public RawImage cameraDisplay;

    [Header("Data Mapping")]
    public float maxRange = 30f;
    public Color minDistanceColor = Color.red;
    public Color maxDistanceColor = Color.blue;

    public void UpdateLiDARVisualization(float[] ranges, Vector3[] directions)
    {
        if (lidarParticleSystem == null) return;

        var main = lidarParticleSystem.main;
        main.maxParticles = ranges.Length;

        var emission = lidarParticleSystem.emission;
        emission.rateOverTime = ranges.Length;

        ParticleSystem.Particle[] particles = new ParticleSystem.Particle[ranges.Length];

        for (int i = 0; i < ranges.Length; i++)
        {
            particles[i].position = robotBase.position +
                                  robotBase.TransformDirection(directions[i]) * ranges[i];
            particles[i].startSize = 0.1f;
            particles[i].startColor = Color.Lerp(minDistanceColor, maxDistanceColor,
                                               ranges[i] / maxRange);
            particles[i].remainingLifetime = 0.5f;
            particles[i].startLifetime = 0.5f;
        }

        lidarParticleSystem.SetParticles(particles, particles.Length);
    }

    public void UpdateCameraFeed(Texture cameraTexture)
    {
        if (cameraFeedTexture != null)
        {
            Graphics.Blit(cameraTexture, cameraFeedTexture);
            cameraDisplay.texture = cameraFeedTexture;
        }
    }
}
```

## Performance Optimization

### Occlusion Culling

Implement occlusion culling for complex scenes:

```csharp
using UnityEngine;

public class OcclusionCullingManager : MonoBehaviour
{
    [Header("Occlusion Settings")]
    public LayerMask occluderLayers;
    public float updateInterval = 0.1f;

    private Camera mainCamera;
    private WaitForSeconds updateDelay;

    void Start()
    {
        mainCamera = Camera.main;
        updateDelay = new WaitForSeconds(updateInterval);

        StartCoroutine(OcclusionUpdateRoutine());
    }

    IEnumerator OcclusionUpdateRoutine()
    {
        while (true)
        {
            UpdateOcclusion();
            yield return updateDelay;
        }
    }

    void UpdateOcclusion()
    {
        // Perform occlusion queries for distant objects
        // Disable rendering for occluded objects
    }
}
```

### Dynamic Batching

Configure Unity for optimal performance with many small objects:

```csharp
using UnityEngine;

public class BatchOptimizer : MonoBehaviour
{
    [Header("Batching Settings")]
    public bool enableDynamicBatching = true;
    public bool enableStaticBatching = true;

    void Start()
    {
        // Static batching is enabled by default
        // Dynamic batching can be configured

        OptimizeMaterials();
    }

    void OptimizeMaterials()
    {
        // Use shared materials to enable batching
        Renderer[] renderers = FindObjectsOfType<Renderer>();

        foreach (Renderer renderer in renderers)
        {
            // Ensure all similar objects use the same material instance
            if (renderer.sharedMaterials.Length > 0)
            {
                Material sharedMat = renderer.sharedMaterials[0];
                renderer.material = sharedMat; // Use instance instead of shared
            }
        }
    }
}
```

## Lighting and Atmospherics

### Realistic Lighting Setup

```csharp
using UnityEngine;

public class DigitalTwinLighting : MonoBehaviour
{
    [Header("Sun Light")]
    public Light sunLight;
    public Gradient dayNightCycle;

    [Header("Atmospheric Scattering")]
    public float atmosphericScattering = 0.1f;

    [Header("Indoor Lighting")]
    public GameObject[] indoorLights;

    void Start()
    {
        SetupLighting();
    }

    void SetupLighting()
    {
        if (sunLight != null)
        {
            sunLight.type = LightType.Directional;
            sunLight.intensity = 1.0f;
            sunLight.shadows = LightShadows.Soft;
            sunLight.shadowStrength = 0.8f;
            sunLight.shadowResolution = ShadowResolution.High;
        }

        // Configure atmospheric effects
        RenderSettings.fog = true;
        RenderSettings.fogMode = FogMode.ExponentialSquared;
        RenderSettings.fogDensity = atmosphericScattering;
    }

    public void SetTimeOfDay(float time) // 0-24 hour format
    {
        float normalizedTime = time / 24f;

        if (sunLight != null)
        {
            // Rotate sun based on time of day
            sunLight.transform.rotation = Quaternion.Euler(
                90 - (normalizedTime * 360) % 180,
                0,
                0
            );

            // Adjust color based on day/night cycle
            sunLight.color = dayNightCycle.Evaluate(normalizedTime);
        }
    }
}
```

## Best Practices for Digital Twin Visualization

1. **Performance First**: Optimize for real-time rendering requirements
2. **Consistent Coordinate Systems**: Match Unity and Gazebo coordinate conventions
3. **Modular Architecture**: Use prefabs and components for reusability
4. **Data-Driven Design**: Separate visualization from simulation data
5. **Quality Settings**: Provide scalable quality options for different hardware
6. **Version Control**: Use Git LFS for large asset files

## Troubleshooting Common Issues

### Rendering Performance
- Use occlusion culling for large environments
- Implement Level of Detail (LOD) systems
- Optimize shader complexity
- Reduce draw calls through batching

### Data Synchronization
- Ensure consistent coordinate systems between Gazebo and Unity
- Handle network latency appropriately
- Validate data integrity and bounds

### Asset Loading
- Preload critical assets during initialization
- Use async loading for large models
- Implement object pooling for frequently instantiated objects

## Next Steps

In the following chapter, we'll explore how to integrate Gazebo and Unity for seamless digital twin operation, covering communication protocols and synchronization mechanisms.