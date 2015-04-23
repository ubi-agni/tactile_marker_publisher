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

import sys
from urdf_parser_py import urdf

xmlr = urdf.xmlr  # define shortcut


def on_error(message):
	sys.stderr.write(message + '\n')


xmlr.core.on_error = on_error

xmlr.start_namespace('urdf')


class Grid(xmlr.Object):
	def __init__(self, size=None, spacing=None, scale=None):
		self.size = size
		self.spacing = spacing
		self.scale = scale if scale is not None else [1, 1]


xmlr.reflect(Grid, params=[
	xmlr.Attribute('size', 'vector2', required=True),
    xmlr.Attribute('spacing', 'vector2', required=True),
	xmlr.Attribute('scale', 'vector2', required=True)
])


class GeometricMarkerType(urdf.GeometricType):
	def __init__(self):
		self.factory = xmlr.FactoryType('markers', {
			'mesh': urdf.Mesh,
			'grid': Grid
		})

xmlr.add_type('markers', GeometricMarkerType())


class TactileMarker(xmlr.Object):
	""" TactileMarker represents the parsed XML structure of <tactile> tags in URDFs """

	def __init__(self, topic=None, data=None, link=None, geometry=None, origin=None,
	             ns='', xs=None, ys=None):
		self.topic = topic
		self.data = data
		self.geometry = geometry
		self.origin = origin
		self.ns = ns
		self.link = link
		self.xs = xs if xs is not None else [None, None]
		self.ys = ys if ys is not None else [0, 1]

	def check_valid(self):
		assert len(self.xs) == len(self.ys), 'ranges xs and ys should have same dimension'
		assert min(self.ys) >= 0 and max(self.ys) <= 1, 'normalization range [0..1] exceeded'


xmlr.reflect(TactileMarker, params=[
	xmlr.Attribute('link', str, required=True),
	xmlr.Attribute('topic', str, required=True),
	xmlr.Attribute('data', str, required=True),
	xmlr.Attribute('ns', str, required=False),
	xmlr.Attribute('xs', 'vector', required=False),
	xmlr.Attribute('ys', 'vector', required=False),
	xmlr.Element('geometry', 'markers'),
	urdf.origin_element,
])


class Robot(urdf.Robot):
	def __init__(self):
		super(Robot, self).__init__()
		self.tactiles = []


xmlr.reflect(Robot, tag='robot', params=[
	xmlr.Attribute('name', str, False),  # Is 'name' a required attribute?
	xmlr.AggregateElement('link', urdf.Link),
	xmlr.AggregateElement('joint', urdf.Joint),
	xmlr.AggregateElement('gazebo', xmlr.RawType()),
	xmlr.AggregateElement('transmission', 'transmission'),
	xmlr.AggregateElement('material', urdf.Material),
	xmlr.AggregateElement('tactile', TactileMarker)
])

# make an alias
URDF = Robot

xmlr.end_namespace()
