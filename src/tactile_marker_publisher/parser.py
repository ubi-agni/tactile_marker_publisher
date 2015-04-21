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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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

from urdf_parser_py.urdf import *
import sys

def on_error(message):
#	sys.stderr.write(message + '\n')
	pass
xmlr.core.on_error = on_error

xmlr.start_namespace('urdf')

class TactileMarker(xmlr.Object):
	""" TactileMarker represents the parsed XML structure of <tactile> tasgs in URDFs """
	def __init__(self, source=None, geometry=None, origin=None, name=None):
		self.source = source
		self.geometry = geometry
		self.origin = origin
		self.name = name
		self.link = None

xmlr.reflect(TactileMarker, params=[
	xmlr.Attribute('source', str, True),
	xmlr.Element('geometry', 'geometric'),
	origin_element,
	name_attribute,
])

# I wasn't able to extend the default URDF parser. Hence, I simply rewrote
# the Link and Robot classes to only included the interesting stuff.
class Link(xmlr.Object):
	def __init__(self, name=None):
		self.aggregate_init()

		self.name = name
		self.tactiles = []

xmlr.reflect(Link, params = [
	name_attribute,
	xmlr.AggregateElement('tactile', TactileMarker)
	])

class Robot(xmlr.Object):
	def __init__(self):
		self.aggregate_init()

		self.links = []

	@classmethod
	def from_parameter_server(cls, key = 'robot_description'):
		"""Retrieve the robot model from parameter server and parse it."""
		import rospy
		return cls.from_xml_string(rospy.get_param(key))

	def getMarkers(self):
		for link in self.links:
			for marker in link.tactiles:
				marker.link = link.name
				yield marker

xmlr.reflect(Robot, tag = 'robot', params = [
	xmlr.AggregateElement('link', Link),
	])

# make an alias
URDF = Robot

xmlr.end_namespace()

