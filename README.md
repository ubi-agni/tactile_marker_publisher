# tactile_marker_publisher

Reads a robot URDF description, having augmented with `<tactile>` tags. 
It subscribes to the specified data stream / topic and republishes tactile pressure values as rviz markers.

The syntax resembles the `<visual>` or `<collision>` tag syntax from `<link>` tags:
```xml
<tactile source="/topic/field[2]/field[3]" link="link_attached_to" name="foo">
	<geometry>
		<mesh filename="package://description/meshes/mesh.stl"
			scale="1 1 1"/>
	</geometry>
	<origin xyz="0 0 0"/>
</tactile>
```

Here `source` describes the topic and data field to listen to - using a syntax that is well known from `rostopic echo`. The tactile marker will be attached to the `link` specified.
