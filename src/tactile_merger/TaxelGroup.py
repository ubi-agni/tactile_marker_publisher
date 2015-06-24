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

import genpy
import std_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker

class TaxelGroup(object):
	def __init__(self, desc, id):
		self.taxels = []
		self.marker = Marker(id=id,
                             ns="force",
							 type=Marker.ARROW,
							 action=Marker.ADD,
							 frame_locked=True,
							 lifetime=genpy.Duration(10))
		self.marker.header.frame_id = desc.link
		self.marker.scale.x, self.marker.scale.y, self.marker.scale.z = desc.geometry.scale[1:]
		self.scale_factor = desc.geometry.scale[0]
		self.marker.points = [geometry_msgs.msg.Point(0,0,0), None]
		self.marker.color = std_msgs.msg.ColorRGBA(*desc.geometry.color.rgba)

	def addTaxel(self, taxel):
		self.taxels.append(taxel)

	def compute(self):
		pos = [0,0,0]
		normal = [0,0,0]
		force = 0
		for taxel in self.taxels:
			w = taxel._weight
			pos += w * taxel._pos
			normal += w * taxel._normal
			force += w
		return force, pos, normal

	def getMarker(self):
		force, start, normal = self.compute()
		if force > 1e-2:
			start = start / force
			end = start + self.scale_factor * normal
			self.marker.points[0] = geometry_msgs.msg.Point(*start)
			self.marker.points[1] = geometry_msgs.msg.Point(*end)
		else:
			self.marker.points[1] = self.marker.points[0]
		return self.marker
