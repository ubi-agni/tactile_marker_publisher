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

from __future__ import print_function
import sys
import threading

import rospy
from rostopic import get_topic_class

import parser
TactileMarkerDesc = parser.TactileMarker
from .marker import *

class Subscriber(object):
	"""
	Subscriber to ROS topic buffering latest tactile data
	A single subscriber can have several markers attached to it,
	corresponding to individual fields in the message
	"""
	factory = {parser.urdf.Mesh: MeshMarker,
			   parser.Grid: PixelGridMarker,
			   parser.Arrow: ArrowMarker}

	def __init__(self, topic):
		"""
		:param topic:       ROS topic name
		:return:            None
		"""
		rospy.loginfo("init subscriber %s", topic)
		self.sub = rospy.Subscriber(topic, rospy.msg.AnyMsg, self._receive_cb, queue_size=2)
		self.lock = threading.Lock()
		self.markers = []
		self.last_any_msg = None
		self.last_typed_msg = None
		self.dirty = False
		self.needsUpdateCalls = False
		self.next_time = rospy.Time()
		self.recv_rate = rospy.Duration(1. / rospy.get_param('~recv_rate', 10))

	def createMarker(self, desc, **kwargs):
		for type, factory in self.factory.iteritems():
			if isinstance(desc.geometry, type):
				return factory(desc, **kwargs)
		raise Exception('unknown marker geometry %s' % desc.geometry)

	def addMarker(self, desc, **kwargs):
		"""
		add marker
		@param TactileMarkerDesc marker	 marker specification
		:rtype : ValueMarker
		"""
		m = self.createMarker(desc, ns=desc.ns, **kwargs)
		self.needsUpdateCalls |= m.needsDataUpdate()

		# add marker to list
		self.lock.acquire()
		self.markers.append(m)
		self.lock.release()

	def close(self):
		self.sub.unregister()

	def _receive_cb(self, any_msg):
		"""
		ROS subscriber callback
		:param any_msg: ROS message data (unserialized)
		"""
		if (rospy.Time.now() < self.next_time):
			return
		self.lock.acquire()

		if self.last_typed_msg is None:
			# retrieve topic type
			data_class, _, _ = get_topic_class(self.sub.name)
			self.last_typed_msg = data_class()

		# store latest message for lazy processing on demand
		self.last_any_msg = any_msg

		if self.needsUpdateCalls:
			self.update()
		self.next_time = rospy.Time.now() + self.recv_rate;
		self.lock.release()

	def config(self, cfg):
		self.lock.acquire()
		ValueMarker.setmode(cfg.mode)
		for m in self.markers:
			m.config(cfg)
		self.lock.release()

	def update(self):
		"""
		update all marker's internal data storage _tactile_data
		deserializing and extracting the data and applying the _update_cb()
		Assumes, that lock was acquired before!
		"""
		if self.last_any_msg is None: return # data already updated

		self.last_typed_msg.deserialize(self.last_any_msg._buff)
		self.last_any_msg = None # indicate successful deserialization
		# iterate over all markers and update their _tactile_data
		for m in self.markers:
			try:
				m.data(self.last_typed_msg)
				self.dirty = True

			except Exception as e:
				print(e, file=sys.stderr)


	def getChangedMarkers(self):
		try:
			self.lock.acquire()
			self.update()

			result = []
			if not self.dirty: return result

			for m in self.markers:
				m.update()
				result.append(m)
			self.dirty = False

			return result

		finally:
			self.lock.release()

