# tactile_marker_publisher

Reads a robot URDF description, having augmented with `<tactile>` tags. 
It subscribes to the specified topic `topic` and re-publishes tactile data as rviz markers on the topic `tactile_markers`.

The syntax resembles the `<visual>` or `<collision>` tag syntax from `<link>` tags:
```xml
<tactile topic="topic" data="field[2]/field[3]" link="link_attached_to" ns="marker namespace" xs="0 4095">
	<geometry>
		<mesh filename="package://description/meshes/mesh.stl" scale="1 1 1"/>
	</geometry>
	<origin xyz="0 0 0" rpy="0 0 0"/>
</tactile>
```

Here `topic` and `data` describe the topic and data field to listen to - using a syntax that is well known from `rostopic echo`. The tactile marker will be attached to the `link` specified.
For proper range normalization you should provide attributes `xs` and `ys` containing lists of mapping values from inputs (`xs`) to normalized outputs (`ys`) in range [0..1]. By default `ys="0 1"`. If neither attribute is provided, the range is auto-estimated from observed values.

Grid-like tactile arrays can be visualized like so:
```xml
<tactile topic="/topic" data="values" link="link_attached_to" ns="marker namespace" xs="0 4095">
	<geometry>
		<grid size="16 16" spacing="0.005 0.005" scale="0.005 0.005"/>
	</geometry>
	<origin xyz="0 0 0.01"/>
</tactile>
```

## parameters

### mode (default: absCurrent)
The `tactile_marker_publisher` supports various visualization and dynamic calibration modes, accessible by parameter mode:
- 0 / rawCurrent: raw value as measured
- 1 / rawMean: raw value averaged over time
- 2 / absCurrent: measured value normalized to observed range
- 3 / absMean: abs averaged over time
- 4 / dynCurrent: measured value normalized to dynamically adapted range
- 5 / dynMean: relative averaged over time
- 6 / dynCurrentRelease: show release as red, press as green
- 7 / dynMeanRelease: release averaged over time

The modes 0-5 employ a colormap from black, over green and yellow, to red.

### value_smoothing (default: 0.7)
In order to compute the mean values a sliding average is used:
```
mean = lambda * mean + (1-lambda) * current
```

Clearly, values closer to 1.0 generate smoother (and more delayed) observations.
Values closer to 0.0 generate observations closer to current data.

### range_habituation (default: 0.9995)
Modes 4-7 employ a dynamic range adaptation: The minimum and maximum dynamic range values exponentially decay towards the current value:
```
  min = current - lambda * (current - min)
  max = current + lambda * (max - current)
```

If smaller / larger values are observed, the min resp. max values of the range are immediately adjusted.

### release_decay (default: 0.05)
For modes 6-7, if the current values drops below the previous value considerably (10% of absolute observed range) we enter `release mode`,
visualizing the amount of drop as a shade of red. In subsequent time steps, this amount linearly decreases by the amount `release_decay * dynamic range`.

## dynamic reconfigure
All parameters can be dynamically adjusted during runtime using the [`dynamic_reconfigure`](http://wiki.ros.org/dynamic_reconfigure) tools.
Particularly, run `rosrun rqt_reconfigure rqt_reconfigure`.
