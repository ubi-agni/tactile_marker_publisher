# Software License Agreement (BSD License)
#
# Copyright (c) 2015, CITEC, Bielefeld University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy as np

import genpy
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from rospy.names import isstring
from tf.transformations import quaternion_from_euler

import msg
import parser
TactileMarkerDesc = parser.TactileMarker

class ColorMap(object):
	def __init__(self, colors):
		colors = map(list, map(self._rgb_tuple_from, colors))
		colors = np.reshape(colors, (-1, 3))
		self._rs = colors[:, 0]
		self._gs = colors[:, 1]
		self._bs = colors[:, 2]
		self._xs = np.linspace(0, 1, len(colors))

	def map(self, value):
		return (np.interp(value, self._xs, self._rs),
		        np.interp(value, self._xs, self._gs),
		        np.interp(value, self._xs, self._bs),
		        1)

	@classmethod
	def _rgb_tuple_from(cls, color):
		if isstring(color):
			try:
				import webcolors

				color = webcolors.hex_to_rgb(color) if color[0] == '#' else webcolors.name_to_rgb(color)
				color = (v / 255. for v in color)  # turn into list and normalize to [0..1]
			except ImportError:
				raise Exception('Cannot understand color names. Please install python-webcolors.')
			except ValueError:
				raise

		elif not isinstance(color, (list, tuple)):
			raise Exception('unknown color specification: %s (expected name or (r,g,b) tuple)' % color)
		return color


class Marker(visualization_msgs.msg.Marker):
	"""
	Extension of visualization_msgs.msg.Marker to hold tactile data fields
	"""
	try:
		defaultColorMap = ColorMap(['black', 'lime', 'yellow', 'red'])
	except Exception:
		defaultColorMap = ColorMap([(0,0,0), (0,1,0), (1,1,0), (1,0,0)])

	def __init__(self, desc, **kwargs):
		assert isinstance(desc, TactileMarkerDesc)
		super(Marker, self).__init__(frame_locked=True, action=Marker.ADD, ns=desc.ns, **kwargs)
		self.header.frame_id = desc.link
		self.lifetime = genpy.Duration(10)

		# store range for normalization
		self.xs, self.ys = desc.xs, desc.ys
		inreasing  = np.all(np.diff(self.xs) > 0)
		decreasing = np.all(np.diff(self.xs) < 0)
		if not inreasing:
			if decreasing:
				# reverse order of xs and ys
				self.xs = list(reversed(self.xs))
				self.ys = list(reversed(self.ys))
			else:
				raise Exception('expected list of strictly increasing or decreasing values')
		self.auto_range = any(v is None for v in self.xs)
		if self.auto_range: self.xs = [float('inf'), float('-inf')]

		# new fields
		self._field_evals = msg.generate_field_evals(desc.data)
		self._tactile_data = None
		self.colorMap = self.defaultColorMap

		# handle optional fields
		if desc.origin:
			if desc.origin.xyz:
				self.pose.position = geometry_msgs.msg.Point(*desc.origin.xyz)
			if desc.origin.rpy:
				self.pose.orientation = geometry_msgs.msg.Quaternion(
					*quaternion_from_euler(*desc.origin.rpy))

	def normalize(self, value):
		if self.auto_range:
			self.xs[0] = min(self.xs[0], value)
			self.xs[1] = max(self.xs[1], value)
		return np.interp(value, self.xs, self.ys)

	def update(self, data):
		pass


class MeshMarker(Marker):
	def __init__(self, desc, **kwargs):
		super(MeshMarker, self).__init__(desc, **kwargs)
		self.type = Marker.MESH_RESOURCE
		self.mesh_resource = desc.geometry.filename
		if desc.geometry.scale:
			self.scale.x, self.scale.y, self.scale.z = desc.geometry.scale

	def update(self, data):
		self.color = std_msgs.msg.ColorRGBA(*self.colorMap.map(self.normalize(data)))


class PixelGridMarker(Marker):
	def __init__(self, desc, **kwargs):
		super(PixelGridMarker, self).__init__(desc, **kwargs)

		self.type = Marker.CUBE_LIST
		g = desc.geometry
		xoff, yoff = [-(n-1)/2. * delta for n, delta in zip(g.size, g.spacing)]
		xdelta, ydelta = g.spacing
		for xi in range(int(g.size[0])):
			for yi in range(int(g.size[1])):
				self.points.append(geometry_msgs.msg.Point(xoff+xi*xdelta,
				                                           yoff+yi*ydelta,
				                                           0))
				self.colors.append(std_msgs.msg.ColorRGBA())
		self.scale.x, self.scale.y = g.scale

	def update(self, data):
		assert len(self.colors) == len(data)
		for color, val in zip(self.colors, data):
			color.r, color.g, color.b, color.a = self.colorMap.map(self.normalize(val))

