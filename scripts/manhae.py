#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

import lanelet2
from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class ManhaeHDMap(Node):
    def __init__(self):
        super().__init__('manhae_hd_map')

        self.marker_pub = self.create_publisher(MarkerArray, '/hd_map_markers', 10)

        self.map_path = None
        self.projector = None
        self.lanelet_map = self.load_map()

        self.map_origin_x = None
        self.map_origin_y = None
        self.set_map_origin()

        self.osm_lines = self.load_osm_lines()
        self.centerlines = self.make_centerlines()

        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('만해광장 HD Map 노드 시작')
        self.get_logger().info(f'lanelet 개수: {len(self.lanelet_map.laneletLayer)}')
        self.get_logger().info(f'OSM 일반 way 개수: {len(self.osm_lines)}')
        self.get_logger().info(f'map origin x={self.map_origin_x:.3f}, y={self.map_origin_y:.3f}')

        for lanelet_id, centerline in self.centerlines.items():
            self.get_logger().info(
                f'lanelet {lanelet_id} centerline point 개수: {len(centerline)}'
            )

    def load_map(self):
        package_share_dir = get_package_share_directory('hd_map')
        self.map_path = os.path.join(package_share_dir, 'maps', '만해광장_연습.osm')

        origin_lat = 37.55955
        origin_lon = 126.99961

        origin = lanelet2.io.Origin(origin_lat, origin_lon)
        self.projector = lanelet2.projection.UtmProjector(origin)

        lanelet_map, errors = lanelet2.io.loadRobust(self.map_path, self.projector)

        if errors:
            for e in errors:
                self.get_logger().warn(str(e))
        else:
            self.get_logger().info('Lanelet2 map 로드 에러 없음')

        return lanelet_map

    def load_osm_lines(self):
        if self.map_path is None or self.projector is None:
            return []

        tree = ET.parse(self.map_path)
        root = tree.getroot()

        nodes = {}
        for node in root.findall('node'):
            node_id = node.attrib.get('id')
            lat = node.attrib.get('lat')
            lon = node.attrib.get('lon')
            if node_id is None or lat is None or lon is None:
                continue

            gps_point = lanelet2.core.GPSPoint(float(lat), float(lon), 0.0)
            map_point = self.projector.forward(gps_point)
            p = Point()
            p.x = float(map_point.x)
            p.y = float(map_point.y)
            p.z = 0.0
            nodes[node_id] = p

        lines = []
        for way in root.findall('way'):
            tags = {
                tag.attrib.get('k'): tag.attrib.get('v')
                for tag in way.findall('tag')
                if tag.attrib.get('k') is not None
            }

            if tags.get('type') == 'line_thin':
                continue

            points = []
            for nd in way.findall('nd'):
                point = nodes.get(nd.attrib.get('ref'))
                if point is not None:
                    points.append(point)

            if len(points) >= 2:
                lines.append({
                    'id': int(way.attrib.get('id', len(lines))),
                    'points': points,
                    'tags': tags,
                })

        return lines

    def set_map_origin(self):
        for ll in self.lanelet_map.laneletLayer:
            p = ll.leftBound[0]
            self.map_origin_x = p.x
            self.map_origin_y = p.y
            return

        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

    def make_centerlines(self):
        centerlines = {}

        for ll in self.lanelet_map.laneletLayer:
            left_bound = ll.leftBound
            right_bound = ll.rightBound

            if len(left_bound) != len(right_bound):
                self.get_logger().warn(
                    f'lanelet {ll.id}: left/right point 개수가 다름. '
                    f'left={len(left_bound)}, right={len(right_bound)}'
                )
                continue

            centerline = []

            for left_p, right_p in zip(left_bound, right_bound):
                center_p = Point()
                center_p.x = float((left_p.x + right_p.x) / 2.0)
                center_p.y = float((left_p.y + right_p.y) / 2.0)
                center_p.z = 0.0
                centerline.append(center_p)

            centerlines[ll.id] = centerline

        return centerlines

    def make_line_marker(self, marker_id, points, ns, r, g, b, width=0.05, z=0.0):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = width

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        for p in points:
            pt = Point()
            pt.x = float(p.x)
            pt.y = float(p.y)
            pt.z = z
            marker.points.append(pt)

        return marker

    def make_osm_line_marker(self, marker_id, osm_line):
        tags = osm_line['tags']
        color = (1.0, 1.0, 1.0)

        if tags.get('highway') == 'service':
            width = 0.1
        elif tags.get('highway') == 'steps':
            width = 0.13
        elif tags.get('natural') is not None:
            width = 0.13
        elif tags.get('leisure') is not None:
            width = 0.13
        else:
            width = 0.13

        marker = self.make_line_marker(
            marker_id,
            osm_line['points'],
            'osm_way',
            color[0], color[1], color[2],
            width=width,
            z=-0.02
        )
        marker.color.a = 0.75
        return marker

    def publish_map(self):
        marker_array = MarkerArray()

        marker_id = 0

        for osm_line in self.osm_lines:
            marker_array.markers.append(
                self.make_osm_line_marker(marker_id, osm_line)
            )
            marker_id += 1

        for ll in self.lanelet_map.laneletLayer:
            left_marker = self.make_line_marker(
                marker_id,
                ll.leftBound,
                'left_bound',
                1.0, 1.0, 0.0,
                width=0.05
            )
            marker_array.markers.append(left_marker)
            marker_id += 1

            right_marker = self.make_line_marker(
                marker_id,
                ll.rightBound,
                'right_bound',
                0.0, 1.0, 1.0,
                width=0.05
            )
            marker_array.markers.append(right_marker)
            marker_id += 1

            if ll.id in self.centerlines:
                center_marker = self.make_line_marker(
                    marker_id,
                    self.centerlines[ll.id],
                    'centerline',
                    1.0, 0.0, 0.0,
                    width=0.025
                )
                center_marker.color.a = 0.5
                marker_array.markers.append(center_marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = ManhaeHDMap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('만해광장 HD Map 노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
