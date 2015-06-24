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
from tactile_marker_publisher.parser import TactileMarker, Arrow
from .subscriber import Subscriber
from .Taxel import Taxel
from .TaxelGroup import TaxelGroup


class Publisher(object):
	def __init__(self, taxels, tf_prefix=''):
		self.subscribers = {}
		self.taxel_groups = {}
		# create a 32bit integer hash
		# - with 15bits in front from the node name
		# - leaving 16bits in the end free for the marker number
		numMarkers = 0
		hash_id = hash(rospy.names.get_name()) & (0x7FFF << 16)
		for desc in taxels:
			assert isinstance(desc, TactileMarker)
			if not isinstance(desc.geometry, Arrow):
				continue

			if desc.topic in self.subscribers:
				sub = self.subscribers[desc.topic]
			else:
				self.subscribers[desc.topic] = sub = Subscriber(desc.topic)

			desc.link = tf_prefix + desc.link

			group_name = desc.ns + "." + desc.link
			if group_name in self.taxel_groups:
				group = self.taxel_groups[group_name]
			else:
				self.taxel_groups[group_name] = group = TaxelGroup(desc, id=hash_id + numMarkers)
			numMarkers += 1

			taxel = Taxel(desc)
			group.addTaxel(taxel)
			sub.addTaxel(taxel)

		self.pub = rospy.Publisher('tactile_markers', MarkerArray, queue_size=1)


	def publish(self):
		"""
		publish array of changed markers
		:return:
		"""
		# update all taxel weights from latest message
		for sub in self.subscribers.itervalues():
			sub.update()
		# fill marker array
		msg = MarkerArray()
		for group in self.taxel_groups.itervalues():
			m = group.getMarker()
			if m: msg.markers.append(m)
		if msg.markers: self.pub.publish(msg)
