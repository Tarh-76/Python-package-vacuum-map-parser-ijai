"""Ijai map parser."""

import logging
import math
import zlib
from typing import Any

from vacuum_map_parser_base.config.color import ColorsPalette
from vacuum_map_parser_base.config.drawable import Drawable
from vacuum_map_parser_base.config.image_config import ImageConfig
from vacuum_map_parser_base.config.size import Sizes
from vacuum_map_parser_base.config.text import Text
from vacuum_map_parser_base.map_data import Area, ImageData, MapData, Path, Point, Room, Wall, Zone
from vacuum_map_parser_base.map_data_parser import MapDataParser
import vacuum_map_parser_ijai.RobotMap_pb2 as RobotMap


from .image_parser import IjaiImageParser
from .aes_decryptor import decrypt

_LOGGER = logging.getLogger(__name__)


class IjaiMapDataParser(MapDataParser):
    """Ijai map parser."""

    POSITION_UNKNOWN = 1100
    robot_map = RobotMap.RobotMap()
    
    def __init__(
        self,
        palette: ColorsPalette,
        sizes: Sizes,
        drawables: list[Drawable],
        image_config: ImageConfig,
        texts: list[Text]
    ):
        super().__init__(palette, sizes, drawables, image_config, texts)
        self._image_parser = IjaiImageParser(palette, image_config, drawables)

    def unpack_map(self, raw_encoded: bytes, *args: Any, **kwargs: Any) -> bytes:
        return zlib.decompress(
            decrypt(
                raw_encoded, 
                kwargs['wifi_sn'], 
                kwargs['owner_id'], 
                kwargs['device_id'], 
                kwargs['model'], 
                kwargs['device_mac']))

    def parse(self, raw: bytes, *args: Any, **kwargs: Any) -> MapData:
        map_data = MapData(0, 1)

        IjaiMapDataParser.robot_map.ParseFromString(raw)

#        if feature_flags & IjaiMapDataParser.FEATURE_ROBOT_STATUS != 0:
#            IjaiMapDataParser.parse_section(buf, 'robot_status', map_id)

        if hasattr(self.robot_map, "mapData"):
            map_data.image, map_data.rooms, map_data.cleaned_rooms = self._parse_image()

        if hasattr(self.robot_map, "historyPose"):
            map_data.path = IjaiMapDataParser._parse_history()

        if hasattr(self.robot_map, "chargeStation"):
            pos_info = self.robot_map.chargeStation
            map_data.charger = Point(x = pos_info.x, y = pos_info.y, a = pos_info.phi * 180 / math.pi)
            _LOGGER.debug("pos: %s", map_data.charger)

#        if feature_flags & IjaiMapDataParser.FEATURE_RESTRICTED_AREAS != 0:
#            map_data.walls, map_data.no_go_areas = IjaiMapDataParser._parse_restricted_areas(buf)

#        if feature_flags & IjaiMapDataParser.FEATURE_CLEANING_AREAS != 0:
#            map_data.zones = IjaiMapDataParser._parse_cleaning_areas(buf)

#        if feature_flags & IjaiMapDataParser.FEATURE_NAVIGATE != 0:
#            map_data.goto = IjaiMapDataParser._parse_position(buf, "pos", with_angle=True)
#            value = buf.get_float32("value")
#            _LOGGER.debug("pos: %s, value: %f", map_data.goto, value)

        if hasattr(self.robot_map, "currentPose"):
            pos_info = self.robot_map.currentPose
            map_data.vacuum_position = Point(x = pos_info.x, y = pos_info.y, a = pos_info.phi * 180 / math.pi)
            _LOGGER.debug("pos: %s", map_data.vacuum_position)

#        if feature_flags & 0x00000800 != 0:
#            IjaiMapDataParser._parse_unknown_section(buf)

        if hasattr(self.robot_map, "mapInfo") and hasattr(self.robot_map, "roomDataInfo") and map_data.rooms is not None:
            IjaiMapDataParser._parse_rooms(map_data.rooms)

#        if feature_flags & 0x00002000 != 0:
#            IjaiMapDataParser._parse_unknown_section(buf)

#        if feature_flags & 0x00004000 != 0:
#            IjaiMapDataParser._parse_room_outlines(buf)

        if map_data.rooms is not None:
            _LOGGER.debug("rooms: %s", [str(room) for number, room in map_data.rooms.items()])
            if map_data.rooms is not None and len(map_data.rooms) > 0 and map_data.vacuum_position is not None:
                vacuum_position_on_image = IjaiMapDataParser._map_to_image(map_data.vacuum_position)
                map_data.vacuum_room = IjaiImageParser.get_current_vacuum_room(self.robot_map.mapData.mapData, vacuum_position_on_image)
                if map_data.vacuum_room is not None:
                    map_data.vacuum_room_name = map_data.rooms[map_data.vacuum_room].name
                _LOGGER.debug("current vacuum room: %s", map_data.vacuum_room)
        return map_data

    @staticmethod
    def _map_to_image(p: Point) -> Point:
        return Point(p.x * 20 + 400, p.y * 20 + 400)

    @staticmethod
    def _image_to_map(x: float) -> float:
        return (x - 400) / 20

    def _parse_image(self) -> tuple[ImageData, dict[int, Room], set[int]]:
        image_left = 0
        image_top = 0
        image_width = self.robot_map.mapHead.sizeX
        image_height = self.robot_map.mapHead.sizeY
        image_size = image_height * image_width
        _LOGGER.debug("width: %d, height: %d", image_width, image_height)
        image, rooms_raw, cleaned_areas, cleaned_areas_layer = self._image_parser.parse(self.robot_map.mapData.mapData, image_width, image_height)
        if image is None:
            image = self._image_generator.create_empty_map_image()
        _LOGGER.debug("img: number of rooms: %d, numbers: %s", len(rooms_raw), rooms_raw.keys())
        rooms = {}
        for number, room in rooms_raw.items():
            rooms[number] = Room(
                IjaiMapDataParser._image_to_map(room[0] + image_left),
                IjaiMapDataParser._image_to_map(room[1] + image_top),
                IjaiMapDataParser._image_to_map(room[2] + image_left),
                IjaiMapDataParser._image_to_map(room[3] + image_top),
                number,
            )
        return (
            ImageData(
                image_size,
                image_top,
                image_left,
                image_height,
                image_width,
                self._image_config,
                image,
                IjaiMapDataParser._map_to_image,
                additional_layers={Drawable.CLEANED_AREA: cleaned_areas_layer},
            ),
            rooms,
            cleaned_areas,
        )

    @staticmethod
    def _parse_history() -> Path:
        path_points = []
        for pt in IjaiMapDataParser.robot_map.historyPose.points:
            # 0: taxi, 1: working
            path_points.append(Point(x = pt.x, y = pt.y))
        return Path(len(path_points), 1, 0, [path_points])

#    @staticmethod
#    def _parse_restricted_areas(buf: ParsingBuffer) -> tuple[list[Wall], list[Area]]:
#        walls = []
#        areas = []
#        buf.skip("unknown1", 4)
#        area_count = buf.get_uint32("area_count")
#        for _ in range(area_count):
#            buf.skip("restricted.unknown1", 12)
#            p1 = IjaiMapDataParser._parse_position(buf, "p1")
#            p2 = IjaiMapDataParser._parse_position(buf, "p2")
#            p3 = IjaiMapDataParser._parse_position(buf, "p3")
#            p4 = IjaiMapDataParser._parse_position(buf, "p4")
#            buf.skip("restricted.unknown2", 48)
#            _LOGGER.debug("restricted: %s %s %s %s", p1, p2, p3, p4)
#            if p1 is not None and p2 is not None and p3 is not None and p4 is not None:
#                if p1 == p2 and p3 == p4:
#                    walls.append(Wall(p1.x, p1.y, p3.x, p3.y))
#                else:
#                    areas.append(Area(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y))
#        return walls, areas

#    @staticmethod
#    def _parse_cleaning_areas(buf: ParsingBuffer) -> list[Zone]:
#        buf.skip("unknown1", 4)
#        area_count = buf.get_uint32("area_count")
#        zones = []
#        for _ in range(area_count):
#            buf.skip("area.unknown1", 12)
#            p1 = IjaiMapDataParser._parse_position(buf, "p1")
#            IjaiMapDataParser._parse_position(buf, "p2")
#            p3 = IjaiMapDataParser._parse_position(buf, "p3")
#            IjaiMapDataParser._parse_position(buf, "p4")
#            buf.skip("area.unknown2", 48)
#            if p1 is not None and p3 is not None:
#                zones.append(Zone(p1.x, p1.y, p3.x, p3.y))
#        return zones

    @staticmethod
    def _parse_rooms(map_data_rooms: dict[int, Room]) -> None:
        map_id = IjaiMapDataParser.robot_map.mapHead.mapHeadId
        for map_data in IjaiMapDataParser.robot_map.mapInfo:
            if (map_data.mapHeadId == map_id):
                current_map = map_data
                break
        map_name = current_map.mapName
#        map_arg = buf.get_uint32("map_arg")
        _LOGGER.debug("map#%d: %s", current_map.mapHeadId, map_name)
        for r in IjaiMapDataParser.robot_map.roomDataInfo:
            if map_data_rooms is not None and r.roomId in map_data_rooms:
                map_data_rooms[r.roomId].name = r.roomName
                map_data_rooms[r.roomId].pos_x = r.roomNamePost.x
                map_data_rooms[r.roomId].pos_y = r.roomNamePost.y

            room_text_pos = Point(r.roomNamePost.x, r.roomNamePost.y)
            _LOGGER.debug("room#%d: %s %s", r.roomId, r.roomName, room_text_pos)

#    @staticmethod
#    def _parse_room_outlines(buf: ParsingBuffer) -> None:
#        buf.skip("unknown1", 51)
#        room_count = buf.get_uint32("room_count")
#        for _ in range(room_count):
#            room_id = buf.get_uint32("room.id")
#            segment_count = buf.get_uint32("room.segment_count")
#            for _ in range(segment_count):
#                buf.skip("unknown2", 5)
#            _LOGGER.debug("room#%d: segment_count: %d", room_id, segment_count)