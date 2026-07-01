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

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/hd_map_markers',
            10
        )

        self.default_speed = 10.0
        self.school_zone_speed = 5.0

        self.map_path = None
        self.projector = None

        self.nodes = {}
        self.ways = {}
        self.relations = {}

        self.lanelet_map = self.load_map()

        self.map_origin_x = None
        self.map_origin_y = None
        self.set_map_origin()

        self.load_osm_elements()

        self.osm_lines = self.make_osm_lines()
        self.stop_lines = self.make_stop_lines()
        self.traffic_lights = self.make_traffic_lights()

        self.centerlines = self.make_centerlines_by_lanelet2()
        self.lanelet_rules = self.analyze_lanelet_rules()

        self.timer = self.create_timer(1.0, self.publish_map)

        self.print_summary()

    def load_map(self):
        package_share_dir = get_package_share_directory('hd_map')
        self.map_path = os.path.join(
            package_share_dir,
            'maps',
            '만해광장_HD_MAP.osm'
        )

        origin_lat = 37.55955
        origin_lon = 126.99961

        origin = lanelet2.io.Origin(origin_lat, origin_lon)
        self.projector = lanelet2.projection.UtmProjector(origin)

        lanelet_map, errors = lanelet2.io.loadRobust(
            self.map_path,
            self.projector
        )

        if errors:
            for e in errors:
                self.get_logger().warn(str(e))
        else:
            self.get_logger().info('Lanelet2 map 로드 에러 없음')

        return lanelet_map

    def load_osm_elements(self):
        """
        OSM 파일을 XML로 직접 읽어서 node, way, relation 정보를 저장한다.

        Lanelet2는 geometry를 읽는 데 사용하고,
        XML 파싱은 lanelet 태그, regulatory_element, stop_line 연결 분석용으로 사용한다.
        """
        if self.map_path is None or self.projector is None:
            return

        tree = ET.parse(self.map_path)
        root = tree.getroot()

        self.nodes.clear()
        self.ways.clear()
        self.relations.clear()

        for node in root.findall('node'):
            node_id = node.attrib.get('id')
            lat = node.attrib.get('lat')
            lon = node.attrib.get('lon')

            if node_id is None or lat is None or lon is None:
                continue

            node_id = int(node_id)

            gps_point = lanelet2.core.GPSPoint(
                float(lat),
                float(lon),
                0.0
            )
            map_point = self.projector.forward(gps_point)

            p = Point()
            p.x = float(map_point.x)
            p.y = float(map_point.y)
            p.z = 0.0

            self.nodes[node_id] = p

        for way in root.findall('way'):
            way_id = int(way.attrib.get('id'))

            tags = {}
            for tag in way.findall('tag'):
                k = tag.attrib.get('k')
                v = tag.attrib.get('v')
                if k is not None:
                    tags[k] = v

            points = []
            node_refs = []

            for nd in way.findall('nd'):
                ref = nd.attrib.get('ref')
                if ref is None:
                    continue

                ref = int(ref)
                node_refs.append(ref)

                if ref in self.nodes:
                    points.append(self.nodes[ref])

            self.ways[way_id] = {
                'id': way_id,
                'tags': tags,
                'points': points,
                'node_refs': node_refs,
            }

        for relation in root.findall('relation'):
            relation_id = int(relation.attrib.get('id'))

            tags = {}
            for tag in relation.findall('tag'):
                k = tag.attrib.get('k')
                v = tag.attrib.get('v')
                if k is not None:
                    tags[k] = v

            members = []
            for member in relation.findall('member'):
                member_type = member.attrib.get('type')
                member_ref = member.attrib.get('ref')
                member_role = member.attrib.get('role')

                if member_ref is None:
                    continue

                members.append({
                    'type': member_type,
                    'ref': int(member_ref),
                    'role': member_role,
                })

            self.relations[relation_id] = {
                'id': relation_id,
                'tags': tags,
                'members': members,
            }

    def set_map_origin(self):
        """
        첫 번째 lanelet의 leftBound 첫 점을 map origin으로 저장한다.

        현재 코드는 origin을 좌표에서 빼지는 않는다.
        즉, 로그 확인용 기준점으로만 사용한다.
        """
        for ll in self.lanelet_map.laneletLayer:
            p = ll.leftBound[0]
            self.map_origin_x = float(p.x)
            self.map_origin_y = float(p.y)
            return

        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

    def make_osm_lines(self):
        """
        일반 OSM way를 RViz 배경선으로 만들기 위한 목록.

        line_thin, stop_line, traffic_light는 따로 처리한다.
        """
        lines = []

        for way_id, way in self.ways.items():
            tags = way['tags']

            if tags.get('type') == 'line_thin':
                continue

            if tags.get('type') == 'stop_line':
                continue

            if tags.get('type') == 'traffic_light':
                continue

            if len(way['points']) >= 2:
                lines.append(way)

        return lines

    def make_stop_lines(self):
        """
        type=stop_line인 way만 따로 모은다.
        """
        stop_lines = {}

        for way_id, way in self.ways.items():
            if way['tags'].get('type') == 'stop_line':
                if len(way['points']) >= 2:
                    stop_lines[way_id] = way

        return stop_lines

    def make_traffic_lights(self):
        """
        type=traffic_light인 way만 따로 모은다.
        """
        traffic_lights = {}

        for way_id, way in self.ways.items():
            if way['tags'].get('type') == 'traffic_light':
                if len(way['points']) >= 1:
                    traffic_lights[way_id] = way

        return traffic_lights

    def make_centerlines_by_lanelet2(self):
        """
        핵심 변경 부분.

        기존 방식:
            leftBound[i]와 rightBound[i]를 평균냄

        변경 방식:
            Lanelet2가 제공하는 ll.centerline을 그대로 사용함

        장점:
            left/right 점 개수가 달라도 centerline을 얻을 수 있음.
        """
        centerlines = {}

        for ll in self.lanelet_map.laneletLayer:
            centerline = []

            try:
                for p in ll.centerline:
                    center_p = Point()
                    center_p.x = float(p.x)
                    center_p.y = float(p.y)
                    center_p.z = 0.0
                    centerline.append(center_p)

            except Exception as e:
                self.get_logger().warn(
                    f'lanelet {ll.id}: Lanelet2 centerline 생성 실패: {e}'
                )
                continue

            if len(centerline) >= 2:
                centerlines[int(ll.id)] = centerline
            else:
                self.get_logger().warn(
                    f'lanelet {ll.id}: centerline point 개수가 부족함'
                )

        return centerlines

    def analyze_lanelet_rules(self):
        """
        lanelet relation 태그를 분석해서 제어에 사용할 정보를 정리한다.

        현재 만해광장 HD Map 기준:
            school_zone=yes  -> target_speed = 5
            regulatory_element -> ref_line -> stop_line -> 일시정지 대상
            traffic_light 상태 정보는 제어에 사용하지 않음
        """
        lanelet_rules = {}

        for relation_id, relation in self.relations.items():
            tags = relation['tags']

            if tags.get('type') != 'lanelet':
                continue

            is_school_zone = tags.get('school_zone') == 'yes'

            if is_school_zone:
                target_speed = self.school_zone_speed
            else:
                target_speed = self.default_speed

            stop_line_ids = []
            regulatory_element_ids = []

            for member in relation['members']:
                if member['type'] != 'relation':
                    continue

                if member['role'] != 'regulatory_element':
                    continue

                reg_id = member['ref']
                regulatory_element_ids.append(reg_id)

                reg = self.relations.get(reg_id)
                if reg is None:
                    continue

                for reg_member in reg['members']:
                    if reg_member['role'] != 'ref_line':
                        continue

                    if reg_member['type'] != 'way':
                        continue

                    stop_line_id = reg_member['ref']
                    stop_line_way = self.ways.get(stop_line_id)

                    if stop_line_way is None:
                        continue

                    if stop_line_way['tags'].get('type') == 'stop_line':
                        stop_line_ids.append(stop_line_id)

            lanelet_rules[relation_id] = {
                'lanelet_id': relation_id,
                'is_school_zone': is_school_zone,
                'target_speed': target_speed,
                'stop_line_ids': stop_line_ids,
                'regulatory_element_ids': regulatory_element_ids,
                'tags': tags,
            }

        return lanelet_rules

    def make_line_marker(self, marker_id, points, ns, r, g, b,
                         width=0.05, z=0.0, alpha=1.0):
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
        marker.color.a = alpha

        marker.pose.orientation.w = 1.0

        for p in points:
            pt = Point()
            pt.x = float(p.x)
            pt.y = float(p.y)
            pt.z = z
            marker.points.append(pt)

        return marker

    def make_points_marker(self, marker_id, points, ns, r, g, b,
                           size=0.12, z=0.12, alpha=1.0):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = size
        marker.scale.y = size

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = alpha

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

        if tags.get('highway') == 'service':
            width = 0.1
        else:
            width = 0.13

        marker = self.make_line_marker(
            marker_id,
            osm_line['points'],
            'osm_way',
            1.0, 1.0, 1.0,
            width=width,
            z=-0.02,
            alpha=0.5
        )

        return marker

    def publish_map(self):
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        marker_id = 0

        for osm_line in self.osm_lines:
            marker_array.markers.append(
                self.make_osm_line_marker(marker_id, osm_line)
            )
            marker_id += 1

        for ll in self.lanelet_map.laneletLayer:
            lanelet_id = int(ll.id)
            rule = self.lanelet_rules.get(lanelet_id, {})

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

            if lanelet_id in self.centerlines:
                centerline = self.centerlines[lanelet_id]

                if rule.get('is_school_zone', False):
                    center_marker = self.make_line_marker(
                        marker_id,
                        centerline,
                        'school_zone_centerline',
                        1.0, 0.0, 1.0,
                        width=0.06,
                        alpha=0.9
                    )
                else:
                    center_marker = self.make_line_marker(
                        marker_id,
                        centerline,
                        'centerline',
                        1.0, 0.0, 0.0,
                        width=0.025,
                        alpha=0.5
                    )

                marker_array.markers.append(center_marker)
                marker_id += 1

                point_marker = self.make_points_marker(
                    marker_id,
                    centerline,
                    'centerline_points',
                    0.0, 1.0, 0.0,
                    size=0.12,
                    z=0.12,
                    alpha=1.0
                )
                marker_array.markers.append(point_marker)
                marker_id += 1

        for stop_line_id, stop_line in self.stop_lines.items():
            stop_marker = self.make_line_marker(
                marker_id,
                stop_line['points'],
                'stop_line',
                1.0, 0.2, 0.0,
                width=0.18,
                z=0.05,
                alpha=1.0
            )
            marker_array.markers.append(stop_marker)
            marker_id += 1

        for traffic_light_id, traffic_light in self.traffic_lights.items():
            if len(traffic_light['points']) >= 2:
                traffic_light_line = self.make_line_marker(
                    marker_id,
                    traffic_light['points'],
                    'traffic_light_line',
                    0.0, 1.0, 0.0,
                    width=0.12,
                    z=0.08,
                    alpha=1.0
                )
                marker_array.markers.append(traffic_light_line)
                marker_id += 1

        self.marker_pub.publish(marker_array)

    def print_summary(self):
        self.get_logger().info('만해광장 HD Map 노드 시작')
        self.get_logger().info(f'OSM path: {self.map_path}')
        self.get_logger().info(f'lanelet 개수: {len(self.lanelet_map.laneletLayer)}')
        self.get_logger().info(f'OSM node 개수: {len(self.nodes)}')
        self.get_logger().info(f'OSM way 개수: {len(self.ways)}')
        self.get_logger().info(f'OSM relation 개수: {len(self.relations)}')
        self.get_logger().info(f'stop line 개수: {len(self.stop_lines)}')
        self.get_logger().info(f'traffic light 개수: {len(self.traffic_lights)}')
        self.get_logger().info(
            f'map origin x={self.map_origin_x:.3f}, '
            f'y={self.map_origin_y:.3f}'
        )

        for lanelet_id, centerline in self.centerlines.items():
            rule = self.lanelet_rules.get(lanelet_id, {})

            self.get_logger().info(
                f'lanelet {lanelet_id}: '
                f'centerline point={len(centerline)}, '
                f'school_zone={rule.get("is_school_zone", False)}, '
                f'target_speed={rule.get("target_speed", self.default_speed)}, '
                f'stop_lines={rule.get("stop_line_ids", [])}'
            )


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
