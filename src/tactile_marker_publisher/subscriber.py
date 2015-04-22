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

import string
import sys
import threading
import numpy as np

import rospy
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from rospy.names import isstring
from rostopic import get_topic_class
from tf.transformations import quaternion_from_euler

from .parser import TactileMarker as TactileMarkerDesc


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

	defaultColorMap = ColorMap(['black', 'lime', 'yellow', 'red'])

	def __init__(self, desc, **kwargs):
		assert isinstance(desc, TactileMarkerDesc)
		super(Marker, self).__init__(frame_locked=True, action=Marker.ADD, ns=desc.ns, **kwargs)
		self.header.frame_id = desc.link

		# new fields
		self._field_evals = generate_field_evals(desc.data)
		self._tactile_data = []
		self.colorMap = self.defaultColorMap

		# handle optional fields
		if desc.origin:
			if desc.origin.xyz:
				self.pose.position = geometry_msgs.msg.Point(*desc.origin.xyz)
			if desc.origin.rpy:
				self.pose.orientation = geometry_msgs.msg.Quaternion(
					*quaternion_from_euler(*desc.origin.rpy))

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
		self.color = std_msgs.msg.ColorRGBA(*self.colorMap.map(data))


class Subscriber(object):
	"""
	Subscriber to ROS topic buffering latest tactile data
	A single subscriber can have several markers attached to it,
	corresponding to individual fields in the message
	"""

	def __init__(self, topic):
		"""
		:param topic:       ROS topic name
		:param topic_type:  type of ROS topic
		:return:            None
		"""
		self.sub = rospy.Subscriber(topic, rospy.msg.AnyMsg, self._receive_cb)
		self.lock = threading.Lock()
		self.markers = []
		self.last_any_msg = None
		self.last_typed_msg = None
		self.dirty = False
		self.update_cb = None

	def addMarker(self, desc, **kwargs):
		"""
		add marker
		@param TactileMarkerDesc marker	 marker specification
		:rtype : Marker
		"""
		m = MeshMarker(desc, **kwargs)
		# add marker to list
		self.lock.acquire()
		self.markers.append(m)
		self.lock.release()

	def close(self):
		self.sub.unregister()

	def _receive_cb(self, any_msg):
		"""
		ROS subscriber callback
		:param msg: ROS message data
		"""
		self.lock.acquire()
		if self.last_typed_msg is None:
			# retrieve topic type
			data_class, _, _ = get_topic_class(self.sub.name)
			self.last_typed_msg = data_class()

		# store latest message for lazy processing on demand
		self.last_any_msg = any_msg

		if self.update_cb:
			self.update()

		self.lock.release()


	def update(self):
		"""
		update all marker's internal data storage _tactileData
		deserializing and extracting the data and applying the _update_cb()
		Assumes, that lock was acquired before!
		"""
		if self.last_any_msg is None: return # data already updated

		self.last_typed_msg.deserialize(self.last_any_msg._buff)
		self.last_any_msg = None # indicate successful deserialization
		# iterate over all markers and update their _tactileData
		for m in self.markers:
			try:
				data = _get_data(self.last_typed_msg, m._field_evals)
				if self.update_cb:
					m._tactileData = self.update_cb(data)
				else:
					m._tactileData = data
				self.dirty = True

			except Exception as e:
				sys.stderr.write('%s\n' % e)


	def getChangedMarkers(self):
		try:
			self.lock.acquire()
			self.update()

			result = []
			if not self.dirty: return result

			for m in self.markers:
				result.append(self.fillMarker(m))
			self.dirty = False

			return result

		finally:
			self.lock.release()

	def fillMarker(self, m):
		"""
		Fill marker structure from tactileData
		:param  Marker m: marker msg
		:return Marker m: updated marker msg
		"""
		m.update(m._tactileData)
		return m


####################################################################################'
# data accessor functions
####################################################################################'
def _get_data(val, field_evals):
	if not field_evals:
		return float(val)
	for f in field_evals:
		val = f(val)
	return float(val)


def _array_eval(field_name, slot_num):
	"""
	:param field_name: name of field to index into, ``str``
	:param slot_num: index of slot to return, ``str``
	:returns: fn(msg_field)->msg_field[slot_num]
	"""

	def fn(f):
		return getattr(f, field_name).__getitem__(slot_num)

	return fn


def _field_eval(field_name):
	"""
	:param field_name: name of field to return, ``str``
	:returns: fn(msg_field)->msg_field.field_name
	"""

	def fn(f):
		return getattr(f, field_name)

	return fn


def generate_field_evals(fields):
	try:
		evals = []
		fields = [f for f in fields.split('/') if f]
		for f in fields:
			if '[' in f:
				field_name, rest = f.split('[')
				slot_num = string.atoi(rest[:rest.find(']')])
				evals.append(_array_eval(field_name, slot_num))
			else:
				evals.append(_field_eval(f))
		return evals
	except Exception, e:
		raise Exception("cannot parse field reference [%s]: %s" % (fields, str(e)))
