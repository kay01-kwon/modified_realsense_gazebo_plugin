# modified_realsense_gazebo_plugin

Herein, the noise is added to D435 sensor.

${r\sim \mathcal{N}(0.0, 1.0)}

${\sigma} = base + scale * ${Z[i]^2}

${Z_{noise}}[i] = Z[i] + r*${\sigma}

## Reference

Refer to the following github sites.

https://github.com/m-tartari/realsense_gazebo_description

https://github.com/m-tartari/realsense_gazebo_plugin

https://github.com/pal-robotics-forks/realsense