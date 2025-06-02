# modified_realsense_gazebo_plugin

Herein, the noise is added to D435 sensor.

${r\sim \mathcal{N}(0.0, 1.0)}$

${\sigma} = base + scale \cdot {Z[i]^2}$

${Z_{noise}}[i] = Z[i] + r\cdot{\sigma}$

# Gazebo Plugin Usage

```
<gazebo>
  <plugin name="camera" filename="librealsense_gazebo_plugin.so">
...
    <noiseBase>0.003</noiseBase>
    <noiseScale>0.005</noiseBase>
...
  </plugin>
</gazebo>
```


## References

Refer to the following github sites.

https://github.com/m-tartari/realsense_gazebo_description

https://github.com/m-tartari/realsense_gazebo_plugin

https://github.com/pal-robotics-forks/realsense