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

import rospy
from visualization_msgs.msg import MarkerArray
from .parser import TactileMarker
from .subscriber import Subscriber

class Publisher(object):
	def __init__(self, markers):
		self.subscribers = {}
		self.numMarkers = 0
		for marker in markers:
			assert isinstance(marker, TactileMarker)

			if marker.topic in self.subscribers:
				sub = self.subscribers[marker.topic]
			else:
				self.subscribers[marker.topic] = sub = Subscriber(marker.topic)

			sub.addMarker(marker, id=self.numMarkers)
			self.numMarkers += 1

		self.pub = rospy.Publisher('tactile_markers', MarkerArray, queue_size=1)

	def publish(self):
		"""
		publish array of changed markers
		:return:
		"""
		msg = MarkerArray()
		for sub in self.subscribers.itervalues():
			msg.markers.extend(sub.getChangedMarkers())
		if msg.markers: self.pub.publish(msg)
