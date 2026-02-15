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

import vacuum_map_parser_ijai.beautify_min as Beautify
import vacuum_map_parser_ijai.RobotMap_pb2 as RobotMap

from .aes_decryptor import decrypt
from .ijai_coordinate_transforms import Transformer
from .image_parser import IjaiImageParser

_LOGGER = logging.getLogger(__name__)


class IjaiMapDataParser(MapDataParser):
    """Ijai map parser."""

    POSITION_UNKNOWN = 1100
    VIRTUALWALL_TYPE_WALL = 2
    VIRTUALWALL_TYPE_NO_MOP = 6
    VIRTUALWALL_TYPE_NO_GO = 3

    # pylint: disable=E1101
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

        self.robot_map.ParseFromString(raw)
        # pylint: disable=W0201
        self.coord_transformer = Transformer(self.robot_map)

        if hasattr(self.robot_map, "mapData"):
            map_data.image, map_data.rooms, map_data.cleaned_rooms = self._parse_image()

        if hasattr(self.robot_map, "historyPose"):
            map_data.path = IjaiMapDataParser._parse_history()

        if hasattr(self.robot_map, "chargeStation"):
            pos_info = self.robot_map.chargeStation
            map_data.charger = Point(
                x=pos_info.x, y=pos_info.y, a=pos_info.phi * 180 / math.pi)
            _LOGGER.debug("pos: %s", map_data.charger)

        if hasattr(self.robot_map, "currentPose"):
            pos_info = self.robot_map.currentPose
            map_data.vacuum_position = Point(
                x=pos_info.x, y=pos_info.y, a=pos_info.phi * 180 / math.pi)
            _LOGGER.debug("pos: %s", map_data.vacuum_position)

        if (
                hasattr(self.robot_map, "mapInfo")
                and hasattr(self.robot_map, "roomDataInfo")
                and map_data.rooms is not None):
            IjaiMapDataParser._parse_rooms(map_data.rooms)

        if hasattr(self.robot_map, "virtualWalls"):
            (map_data.walls,
             map_data.no_go_areas,
             map_data.no_mopping_areas) = IjaiMapDataParser._parse_restricted_areas()

        if hasattr(self.robot_map, "areasInfo"):
            map_data.zones = IjaiMapDataParser._parse_cleaning_zones()

        if hasattr(self.robot_map, "navigationPoints"):
            map_data.goto = IjaiMapDataParser._parse_goto_point()

        if map_data.rooms is not None:
            _LOGGER.debug("rooms: %s", [str(room)
                          for number, room in map_data.rooms.items()])
            if map_data.rooms is not None and len(map_data.rooms) > 0 and map_data.vacuum_position is not None:
                vacuum_position_on_image = self.coord_transformer.map_to_image(
                    map_data.vacuum_position)
                map_data.vacuum_room = IjaiImageParser.get_current_vacuum_room(
                    self.robot_map.mapData.mapData, vacuum_position_on_image, IjaiMapDataParser.robot_map.mapHead.sizeX)
                if map_data.vacuum_room is not None:
                    map_data.vacuum_room_name = map_data.rooms[map_data.vacuum_room].name
                _LOGGER.debug("current vacuum room: %s", map_data.vacuum_room)

        if map_data.image is not None and not map_data.image.is_empty:
            self._image_generator.draw_map(map_data)

        return map_data

    def _parse_image(self) -> tuple[ImageData, dict[int, Room], set[int]]:
        image_left = 0
        image_top = 0
        image_width = self.robot_map.mapHead.sizeX
        image_height = self.robot_map.mapHead.sizeY
        image_size = image_height * image_width
        _LOGGER.debug("width: %d, height: %d", image_width, image_height)

        # Non painted map tranformation
        if (
                len(set(self.robot_map.mapData.mapData).symmetric_difference(
                    [0, 128, 127])) == 0
                and len(self.robot_map.roomChain) > 0
                and self.robot_map.mapType == 0):
            buautify_obj = Beautify.BeautifyMap(self.robot_map.mapHead)
            buautify_obj.setMap(self.robot_map.mapData)
            buautify_obj.transform()
            buautify_obj.roomColorByChain(self.robot_map.roomChain)
            buautify_obj.fillInternalObstacles()
            buautify_obj.normalizeMap()
            self.robot_map.mapData.mapData = bytes(buautify_obj.getMap())

        image, rooms_raw, cleaned_areas = self._image_parser.parse(
            self.robot_map.mapData.mapData, image_width, image_height)
        if image is None:
            image = self._image_generator.create_empty_map_image()
        _LOGGER.debug("img: number of rooms: %d, numbers: %s",
                      len(rooms_raw), rooms_raw.keys())
        rooms = {}
        for number, room in rooms_raw.items():
            rooms[number] = Room(
                self.coord_transformer.image_to_map_x(room[0] + image_left),
                self.coord_transformer.image_to_map_y(room[1] + image_top),
                self.coord_transformer.image_to_map_x(room[2] + image_left),
                self.coord_transformer.image_to_map_y(room[3] + image_top),
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
                self.coord_transformer.map_to_image,
            ),
            rooms,
            cleaned_areas,
        )

    @staticmethod
    def _parse_history() -> Path:
        path_points = []
        for pt in IjaiMapDataParser.robot_map.historyPose.points:
            # 0: taxi, 1: working
            path_points.append(Point(x=pt.x, y=pt.y))
        return Path(len(path_points), 1, 0, [path_points])

    @staticmethod
    def _parse_restricted_areas() -> tuple[list[Wall], list[Area], list[Area]]:
        walls = []
        no_go_areas = []
        no_mop_areas = []

        for virtualWall in IjaiMapDataParser.robot_map.virtualWalls:
            p1, p2, p3, p4 = virtualWall.points

            if virtualWall.type == IjaiMapDataParser.VIRTUALWALL_TYPE_WALL:
                walls.append(Wall(p1.x, p1.y, p3.x, p3.y))
            elif virtualWall.type == IjaiMapDataParser.VIRTUALWALL_TYPE_NO_GO:
                no_go_areas.append(
                    Area(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y))
            elif virtualWall.type == IjaiMapDataParser.VIRTUALWALL_TYPE_NO_MOP:
                no_mop_areas.append(
                    Area(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y))
        return walls, no_go_areas, no_mop_areas

    @staticmethod
    def _parse_cleaning_zones() -> list[Zone]:
        zones = []
        for areaInfo in IjaiMapDataParser.robot_map.areasInfo:
            zones.append(Zone(areaInfo.points[0].x,
                              areaInfo.points[0].y,
                              areaInfo.points[2].x,
                              areaInfo.points[2].y))
        return zones

    @staticmethod
    def _parse_goto_point() -> Point | None:
        for navigationPoint in IjaiMapDataParser.robot_map.navigationPoints:
            if (
                    navigationPoint.status == 0
                    and navigationPoint.pointType == 1
                    and navigationPoint.x != 1100.0  # outside map
                    and navigationPoint.y != 1100.0):
                return Point(navigationPoint.x,
                             navigationPoint.y,
                             navigationPoint.phi * 180 / math.pi)
        return None

    @staticmethod
    def _parse_rooms(map_data_rooms: dict[int, Room]) -> None:
        map_id = IjaiMapDataParser.robot_map.mapHead.mapHeadId
        for map_data in IjaiMapDataParser.robot_map.mapInfo:
            if map_data.mapHeadId == map_id:
                current_map = map_data
                break
        map_name = current_map.mapName
        _LOGGER.debug("map#%d: %s", current_map.mapHeadId, map_name)
        for r in IjaiMapDataParser.robot_map.roomDataInfo:
            if map_data_rooms is not None and r.roomId in map_data_rooms:
                map_data_rooms[r.roomId].name = r.roomName
                map_data_rooms[r.roomId].pos_x = r.roomNamePost.x
                map_data_rooms[r.roomId].pos_y = r.roomNamePost.y

            room_text_pos = Point(r.roomNamePost.x, r.roomNamePost.y)
            _LOGGER.debug("room#%d: %s %s", r.roomId,
                          r.roomName, room_text_pos)
