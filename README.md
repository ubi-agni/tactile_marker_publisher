# tactile_marker_publisher

Reads a robot URDF description, having augmented with `<tactile>` tags. 
It subscribes to the specified data stream / topic and re-publishes tactile data as rviz markers.

The syntax resembles the `<visual>` or `<collision>` tag syntax from `<link>` tags:
```xml
<tactile topic="/topic" data="field[2]/field[3]" link="link_attached_to" ns="marker namespace" xs="0 4095">
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
