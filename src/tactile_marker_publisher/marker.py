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

import numpy
import genpy
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from rospy.names import isstring
from tf.transformations import quaternion_from_euler

import msg
from parser import TactileMarker as TactileMarkerDesc
from tactile import TactileValue

class ColorMap(object):
	def __init__(self, colors, min=0, max=1):
		colors = map(list, map(self._rgb_tuple_from, colors))
		colors = numpy.reshape(colors, (-1, 3))
		self._rs = colors[:, 0]
		self._gs = colors[:, 1]
		self._bs = colors[:, 2]
		self._xs = numpy.linspace(min, max, len(colors))

	def map(self, value):
		return (numpy.interp(value, self._xs, self._rs),
		        numpy.interp(value, self._xs, self._gs),
		        numpy.interp(value, self._xs, self._bs),
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


class MarkerInterface(visualization_msgs.msg.Marker):
	"""
	Extension of visualization_msgs.msg.Marker to hold tactile data fields.
	For initialization, an instance of parser.TactileMarker is provided.
	For every incoming message, data(msg) will be called.
	To prepare publishing of a marker, update() will be called.
	"""
	def __init__(self, desc, **kwargs):
		assert isinstance(desc, TactileMarkerDesc)
		super(MarkerInterface, self).__init__(**kwargs)
		self.action = action = visualization_msgs.msg.Marker.ADD
		self.header.frame_id = desc.link   # place marker relative to this link
		self.frame_locked    = True        # attach marker to the link
		self.lifetime = genpy.Duration(10) # undisplay marker after n seconds

		# handle optional origin field
		if desc.origin:
			if desc.origin.xyz:
				self.pose.position = geometry_msgs.msg.Point(*desc.origin.xyz)
			if desc.origin.rpy:
				self.pose.orientation = geometry_msgs.msg.Quaternion(
					*quaternion_from_euler(*desc.origin.rpy))

	def needsDataUpdate(self):
		"""
		Indicates whether the marker class requires data() calls for each
		incoming message, e.g. to normalize data. If not, data() is called only
		once with the most recently received msg just before the update() call.
		"""
		return False

	def config(self, cfg):
		pass

	def data(self, msg):
		pass

	def update(self):
		pass


class PieceWiseLinearCalibration(object):
	def __init__(self, xs, ys):
		self.xs = xs
		self.ys = ys

		# interp can only handle increasing xs list: reverse if necessary
		inreasing  = numpy.all(numpy.diff(xs) > 0)
		decreasing = numpy.all(numpy.diff(xs) < 0)
		if not inreasing:
			if decreasing:
				# reverse order of xs and ys
				self.xs = list(reversed(self.xs))
				self.ys = list(reversed(self.ys))
			else:
				raise Exception('expected list of strictly increasing or decreasing values')

	def apply(self, values):
		return numpy.interp(values, self.xs, self.ys)


class ValueMarker(MarkerInterface):
	"""
	Base class for single-value based markers.
	"""
	try:
		positiveColorMap = ColorMap(['black', 'lime', 'yellow', 'red'], min=0, max=1)
		negativeColorMap = ColorMap(['red', 'black', 'lime'], min=-1, max=1)
	except Exception:
		positiveColorMap = ColorMap([(0,0,0), (0,1,0), (1,1,0), (1,0,0)], min=0, max=1)
		negativeColorMap = ColorMap([(1,0,0), (0,0,0), (0,1,0)], min=-1, max=1)

	mode     = TactileValue.relCurrent
	colorMap = positiveColorMap

	@staticmethod
	def setmode(mode):
		ValueMarker.mode = mode
		if mode >= TactileValue.relCurrentRelease:
			ValueMarker.colorMap = ValueMarker.negativeColorMap
		else:
			ValueMarker.colorMap = ValueMarker.positiveColorMap

	def __init__(self, desc, **kwargs):
		assert isinstance(desc, TactileMarkerDesc)
		super(ValueMarker, self).__init__(desc, **kwargs)

		# new fields
		self._normalization = PieceWiseLinearCalibration(desc.xs, desc.ys)
		self._field_evals = msg.generate_field_evals(desc.data)
		self._tactile_data = TactileValue(max = max(desc.ys))

	def needsDataUpdate(self):
		return True

	def config(self, cfg):
		self._tactile_data.mean_lambda = cfg.value_smoothing
		self._tactile_data.range_lambda = 1.0 - cfg.range_habituation
		self._tactile_data.release_lambda = cfg.release_decay

	def data(self, msg_data):
		d = msg.extract_data(msg_data, self._field_evals)
		d = self._normalization.apply(d)
		self._tactile_data.update(d)


class MeshMarker(ValueMarker):
	def __init__(self, desc, **kwargs):
		super(MeshMarker, self).__init__(desc, **kwargs)

		self.type = ValueMarker.MESH_RESOURCE
		self.mesh_resource = desc.geometry.filename
		if desc.geometry.scale:
			self.scale.x, self.scale.y, self.scale.z = desc.geometry.scale

	def update(self):
		self.color = std_msgs.msg.ColorRGBA(*self.colorMap.map(self._tactile_data.value(self.mode)))


class PixelGridMarker(ValueMarker):
	def __init__(self, desc, **kwargs):
		super(PixelGridMarker, self).__init__(desc, **kwargs)

		self.type = ValueMarker.CUBE_LIST
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

	def update(self):
		for color, val in zip(self.colors, self._tactile_data.value(self.mode)):
			color.r, color.g, color.b, color.a = self.colorMap.map(val)
