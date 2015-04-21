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
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#	contributors may be used to endorse or promote products derived
#	from this software without specific prior written permission.
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
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import roslib.message
import rospy

from rqt_plot.rosplot import get_topic_type
from urdf_parser_py import urdf
from .parser import TactileMarker as TactileMarkerDesc


class Marker(visualization_msgs.msg.Marker):
	"""
	Extension of visualization_msgs.msg.Marker to hold tactile data fields
	"""

	def __init__(self, desc, **kwargs):
		assert isinstance(desc, TactileMarkerDesc)
		kwargs.update(frame_locked=True, action=Marker.ADD)
		super(Marker, self).__init__(**kwargs)
		self.header.frame_id = desc.link

		# new fields
		_, _, fields = get_topic_type(desc.source)
		self._field_evals = generate_field_evals(fields)
		self._tactile_data = []
		self.dirty = False

		# handle general fields
		if desc.name: self.text = desc.name
		if desc.origin:
			if desc.origin.xyz:
				self.pose.position = geometry_msgs.msg.Point(*desc.origin.xyz)
			if desc.origin.rpy:
				# TODO: Is there a function to transform rpy into quaternion?
				self.pose.orientation = geometry_msgs.msg.Quaternion(desc.origin.rpy)

	def update(self):
		pass


class MeshMarker(Marker):
	def __init__(self, desc, **kwargs):
		super(MeshMarker, self).__init__(desc, **kwargs)
		self.type = Marker.MESH_RESOURCE
		self.mesh_resource = desc.geometry.filename
		if desc.geometry.scale:
			self.scale.x, self.scale.y, self.scale.z = desc.geometry.scale

	def update(self, data):
		self.color = colorMapping(data)


class Subscriber(object):
	"""
	Subscriber to ROS topic buffering latest tactile data
	A single subscriber can have several markers attached to it,
	corresponding to individual fields in the message
	"""

	def __init__(self, topic, topic_type):
		"""
		:param topic:       ROS topic name
		:param topic_type:  type of ROS topic
		:return:            None
		"""
		self.lock = threading.Lock()
		msg_class = roslib.message.get_message_class(topic_type)
		self.sub = rospy.Subscriber(topic, msg_class, self._receive_cb)
		self.markers = []

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

	def _receive_cb(self, msg):
		"""
		ROS subscriber callback
		:param msg: ROS message data
		"""
		try:
			self.lock.acquire()
			try:
				# iterate over all markers on this topic
				for m in self.markers:
					m._tactileData = _get_data(msg, m._field_evals)
					m.dirty = True

			except Exception as e:
				sys.stderr.write('%s\n' % e)

		finally:
			self.lock.release()

	def getChangedMarkers(self):
		result = []
		try:
			self.lock.acquire()
			for m in self.markers:
				if not m.dirty: continue
				result.append(self.fillMarker(m))
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
		m.dirty = False
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

def colorMapping(value):
	if value > 4095: value = 4095
	if value <= 1365:
		R = 0
		G = 0x100 * (1. if 1000*value / 5353 > 255 else 1000*value / 1365015.0)
	else:
		if value <= 2730:
			R = 1000 * (value-1365) / 1365015.0
			G = 1.
		else:
			R = 1.
			G = 1. - ((1000*(value-2730) / 1365015.0))

	return std_msgs.msg.ColorRGBA(r=R, g=G, b=0, a=1)