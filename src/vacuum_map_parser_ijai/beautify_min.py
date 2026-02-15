# pylint: skip-file
import collections

from vacuum_map_parser_base.map_data import Point

from vacuum_map_parser_ijai.RobotMap_pb2 import RobotMap


class BeautifyMap:
    def __init__(self, mapHead: RobotMap.MapHeadInfo):
        self.map = []
        self.tRect = {
            "x": 0,
            "y": 0,
            "width": 0,
            "height": 0
        }
        self.x_min = mapHead.minX
        self.x_max = mapHead.maxX
        self.y_min = mapHead.minY
        self.y_max = mapHead.maxY
        self.resolution = mapHead.resolution
        self.size_x = mapHead.sizeX
        self.size_y = mapHead.sizeY

    def setMap(self, mapData: RobotMap.MapDataInfo):
        temp_mapData = mapData.mapData

        tempArray = [0] * len(temp_mapData)

        for i in range(len(temp_mapData)):
            if (temp_mapData[i] > 127):
                tempArray[i] = -128
            else:
                tempArray[i] = temp_mapData[i]

        self.map = tempArray

    def normalizeMap(self) -> None:
        # normalizing all data to bytes and values suitable for map_data_parser
        for i in range(len(self.map)):
            if self.map[i] < 0:
                self.map[i] = (256 + self.map[i]) % 256
            elif self.map[i] > 255:
                self.map[i] = self.map[i] % 256
            elif self.map[i] == 30:
                self.map[i] = 0
            elif self.map[i] == 40:
                self.map[i] = 255

    def getMap(self) -> list[int]:
        return self.map

    def transform(self) -> None:
        non_boundary_noise = []
        self.findRoiMap()
        self.expandBlackRect(4, 4, self.map[0])
        self.expandWhiteRect(4, 4, self.map[0])
        self.refineBoundary(0, 10)
        non_boundary_noise = self.eliminateNonBoundaryNoise(
            non_boundary_noise, 127, -128, 0)
        self.expandSingleConvexBoundary(50, -128, 4, 4)
        non_boundary_noise = self.fillNonBoundaryNoise2(
            non_boundary_noise)
        self.refineBoundary(0, 10)
        self.fillBlackComponent([], -128)

    def findRoiMap(self):
        top_bound = self.size_x
        bottom_bound = 0
        left_bound = self.size_y
        right_bound = 0
        for x in range(self.size_x):
            for y in range(self.size_y):
                if (self.map[y * self.size_x + x] != 0):
                    if (left_bound > y - 10):
                        if (x - 10 >= 0):
                            left_bound = y - 10
                        else:
                            left_bound = 0
                    if (top_bound > x - 10):
                        if (x - 10 >= 0):
                            top_bound = x - 10
                        else:
                            top_bound = 0
                    if (right_bound < y + 10):
                        if (y + 10 < self.size_y):
                            right_bound = y + 10
                        else:
                            right_bound = self.size_y - 1
                    if (bottom_bound < x + 10):
                        if (x + 10 < self.size_x):
                            bottom_bound = x + 10
                        else:
                            bottom_bound = self.size_x - 1

        width = right_bound - left_bound + 1
        height = bottom_bound - top_bound + 1
        if (width > 0 and height > 0 and width < self.size_y and height < self.size_x):
            self.tRect["x"] = top_bound
            self.tRect["y"] = left_bound
            self.tRect["width"] = width
            self.tRect["height"] = height

    def expandBlackRect(self, kernel_size_x, kernel_size_y, threshold):
        il, ir, jl, jr = (None, None, None, None)

        if (kernel_size_x % 2 == 1):
            ir = kernel_size_x - 1 >> 1
            il = -ir
        else:
            ir = kernel_size_x >> 1
            il = 1 - ir

        if (kernel_size_y % 2 == 1):
            jr = kernel_size_y - 1 >> 1
            jl = -jr
        else:
            jr = kernel_size_y >> 1
            jl = 1 - jr

        dst = [127] * len(self.map)

        for i in range(self.tRect["y"], self.tRect["y"] + self.tRect["width"]):
            for j in range(self.tRect["x"], self.tRect["x"] + self.tRect["height"]):
                if (self.map[i * self.size_x + j] < threshold):
                    for di in range(il, ir + 1):
                        for dj in range(jl, jr + 1):
                            if (i + di < 0 or i + di >= self.tRect["y"] + self.tRect["width"] or j + dj < 0 or j + dj >= self.tRect["x"] + self.tRect["height"]):
                                continue

                            if (dst[(i + di) * self.size_x + j + dj] > self.map[i * self.size_x + j]):
                                dst[(i + di) * self.size_x + j +
                                    dj] = self.map[i * self.size_x + j]

        for offset in range(len(self.map)):
            if (dst[offset] == 127):
                dst[offset] = self.map[offset]

        self.map = dst

    def expandWhiteRect(self, kernel_size_x, kernel_size_y, threshold):
        il, ir, jl, jr = (None, None, None, None)

        if (kernel_size_x % 2 == 1):
            ir = kernel_size_x - 1 >> 1
            il = -ir
        else:
            ir = kernel_size_x >> 1
            il = 1 - ir

        if (kernel_size_y % 2 == 1):
            jr = kernel_size_y - 1 >> 1
            jl = -jr
        else:
            jr = kernel_size_y >> 1
            jl = 1 - jr

        dst = [-128] * len(self.map)

        for i in range(self.tRect["y"], self.tRect["y"] + self.tRect["width"]):
            for j in range(self.tRect["x"], self.tRect["x"] + self.tRect["height"]):
                if (self.map[i * self.size_x + j] > threshold):
                    for di in range(il, ir + 1):
                        for dj in range(jl, jr + 1):
                            if (i + di < 0 or i + di >= self.tRect["y"] + self.tRect["width"] or j + dj < 0 or j + dj >= self.tRect["x"] + self.tRect["height"]):
                                continue

                            if (dst[(i + di) * self.size_x + j + dj] < self.map[i * self.size_x + j] and self.map[(i + di) * self.size_x + j + dj] < threshold):
                                dst[(i + di) * self.size_x + j +
                                    dj] = self.map[i * self.size_x + j]

        for offset in range(len(self.map)):
            if (dst[offset] == -128):
                dst[offset] = self.map[offset]

        self.map = dst

    def refineBoundary(self, threshold_black, threshold_white):
        points = []
        hasWhiteNeighbor = None

        for i in range(self.tRect["y"], self.tRect["y"] + self.tRect["width"]):
            for j in range(self.tRect["x"], self.tRect["x"] + self.tRect["height"]):
                if (self.map[i * self.size_x + j] < threshold_black):
                    hasWhiteNeighbor = False

                    for di in range(-1, 2):
                        for dj in range(-1, 2):
                            if (i + di < 0 or i + di >= self.tRect["y"] + self.tRect["width"] or j + dj < 0 or j + dj >= self.tRect["x"] + self.tRect["height"]):
                                continue

                            if (self.map[(i + di) * self.size_x + j + dj] > threshold_white):
                                hasWhiteNeighbor = True

                    if (not hasWhiteNeighbor):
                        points.append((i, j))

        for x, y in points:
            self.map[x * self.size_x + y] = 0

    def eliminateNonBoundaryNoise(self, nonBoundaryNoise, noise_color, border_color, outer_border_color):
        tempnonBoundaryNoise = nonBoundaryNoise

        for i in range(self.tRect["y"], self.tRect["y"] + self.tRect["width"]):
            for j in range(self.tRect["x"], self.tRect["x"] + self.tRect["height"]):
                if (self.map[i * self.size_x + j] == border_color):
                    if (i - 1 < 0 or i + 1 >= self.tRect["y"] + self.tRect["width"] or j - 1 < 0 or j + 1 >= self.tRect["x"] + self.tRect["height"]):
                        continue

                    if (self.map[(i - 1) * self.size_x + j] != outer_border_color and self.map[(i + 1) * self.size_x + j] != outer_border_color and self.map[i * self.size_x + j - 1] != outer_border_color and self.map[i * self.size_x + j + 1] != outer_border_color and self.map[(i - 1) * self.size_x + j - 1] != outer_border_color and self.map[(i - 1) * self.size_x + j + 1] != outer_border_color and self.map[(i + 1) * self.size_x + j - 1] != outer_border_color and self.map[(i + 1) * self.size_x + j + 1] != outer_border_color):
                        self.map[i * self.size_x + j] = noise_color
                        tempnonBoundaryNoise.append(Point(i, j))

        return tempnonBoundaryNoise

    def expandSingleConvexBoundary(self, external_corner_value, fill_value, valid_length, times):
        contour = self.extractExternalContoursNewStrategy([])

        for _ in range(times):
            fill_edges = []
            inner_corner_value = external_corner_value + 5
            four_neighbourhood = [[-1, 0], [1, 0], [0, -1], [0, 1]]

            extract_corner = self.extractCorners(
                [], contour, external_corner_value, inner_corner_value)

            for p in extract_corner:
                is_valid_length = False

                for k in range(4):
                    currpoint = Point(
                        p.x + four_neighbourhood[k][0], p.y + four_neighbourhood[k][1])

                    if (currpoint.x < self.tRect["y"] or currpoint.x >= self.tRect["y"] + self.tRect["width"] or currpoint.y < self.tRect["x"] or currpoint.y >= self.tRect["x"] + self.tRect["height"]):
                        continue

                    if (self.map[currpoint.x * self.size_x + currpoint.y] == inner_corner_value):
                        is_valid_length = self.statisticalLineLength(
                            currpoint, external_corner_value, inner_corner_value, valid_length)
                        break

                    if (k == 3):
                        is_valid_length = True

                _, fill_edges = self.fourNeighbourhoodSearchForExtractCorners(
                    p, [], [], external_corner_value, inner_corner_value, valid_length, is_valid_length)

            contour = self.fillEdges(contour, fill_edges, fill_value)

    def extractExternalContoursNewStrategy(self, contour):
        gray_region = self.findGrayConnectComponent([])
        return self.findExternalContoursNewStrategy(gray_region, contour)

    def findGrayConnectComponent(self, gray_region) -> tuple[list[Point], list[int]]:
        four_neighbourhood = [[-1, 0], [0, 1], [1, 0], [0, -1]]
        findOnePoint = False

        for y in range(self.tRect["y"], self.tRect["y"] + self.tRect["width"]):
            for x in range(self.tRect["x"], self.tRect["x"] + self.tRect["height"]):
                if (self.map[y * self.size_x + x] == 0):
                    findOnePoint = True
                    points_for_search = [Point(y, x)]
                    gray_region.append(Point(y, x))
                    self.map[y * self.size_x + x] = 30

                    while (len(points_for_search) > 0):
                        seed = points_for_search.pop(0)

                        for k in range(4):
                            currpoint = Point(
                                seed.x + four_neighbourhood[k][0], seed.y + four_neighbourhood[k][1])

                            if (currpoint.x < self.tRect["y"] or currpoint.x >= self.tRect["y"] + self.tRect["width"] or currpoint.y < self.tRect["x"] or currpoint.y >= self.tRect["x"] + self.tRect["height"]):
                                continue

                            if (self.map[currpoint.x * self.size_x + currpoint.y] == 0):
                                self.map[currpoint.x *
                                         self.size_x + currpoint.y] = 30
                                points_for_search.append(currpoint)
                                gray_region.append(currpoint)

                if findOnePoint:
                    break

            if findOnePoint:
                findOnePoint = False
                break

        return gray_region

    def findExternalContoursNewStrategy(self, gray_region, contour) -> list[Point]:
        eight_neighbourhood = [[-1, 0], [1, 0], [0, -1],
                               [0, 1], [-1, 1], [1, 1], [1, -1], [-1, -1]]

        for i in range(len(gray_region)):
            for k in range(8):
                temp_idy = gray_region[i].x + eight_neighbourhood[k][0]
                temp_idx = gray_region[i].y + eight_neighbourhood[k][1]

                if (temp_idy < self.tRect["y"] or temp_idy >= self.tRect["y"] + self.tRect["width"] or temp_idx < self.tRect["x"] or temp_idx >= self.tRect["x"] + self.tRect["height"]):
                    continue

                if (self.map[temp_idy * self.size_x + temp_idx] == -128):
                    self.map[temp_idy * self.size_x + temp_idx] = 40
                    contour.append(Point(temp_idy, temp_idx))

        return contour

    def extractCorners(self, extract_corner, contour, external_corner_value, inner_corner_value):
        four_neighbourhood = [[-1, 0], [0, 1], [1, 0], [0, -1]]

        for i in range(len(contour)):
            black_count = 0
            white_count = 0
            gray_count = 0

            for k in range(4):
                currpoint = Point(
                    contour[i].x + four_neighbourhood[k][0], contour[i].y + four_neighbourhood[k][1])

                if (currpoint.x < self.tRect["y"] or currpoint.x >= self.tRect["y"] + self.tRect["width"] or currpoint.y < self.tRect["x"] or currpoint.y >= self.tRect["x"] + self.tRect["height"]):
                    continue

                if (self.map[currpoint.x * self.size_x + currpoint.y] == -128):
                    black_count += 1
                elif (self.map[currpoint.x * self.size_x + currpoint.y] == 0):
                    gray_count += 1
                elif (self.map[currpoint.x * self.size_x + currpoint.y] == 127):
                    white_count += 1

                if (gray_count == 2 and black_count == 2):
                    extract_corner.append(currpoint)
                    self.map[contour[i].x * self.size_x +
                             contour[i].y] = external_corner_value
                elif (white_count == 2 and black_count == 2):
                    self.map[contour[i].x * self.size_x +
                             contour[i].y] = inner_corner_value

        return extract_corner

    def statisticalLineLength(self, point, external_corner_value, inner_corner_value, valid_length):
        if (self.upSearchStatisticalLineLength(point, external_corner_value, inner_corner_value, valid_length)):
            return True
        elif (self.downSearchStatisticalLineLength(point, external_corner_value, inner_corner_value, valid_length)):
            return True
        elif (self.leftSearchStatisticalLineLength(point, external_corner_value, inner_corner_value, valid_length)):
            return True
        elif (self.rightSearchStatisticalLineLength(point, external_corner_value, inner_corner_value, valid_length)):
            return True

        return False

    def upSearchStatisticalLineLength(self, point, external_corner_value, inner_corner_value, valid_length):
        if (point.x + 1 < self.tRect["y"] + self.tRect["width"] and self.map[(point.x + 1) * self.size_x + point.y] == 127):
            idy = point.x + 1
            idx = point.y
            line = []
            line.append(Point(idy, idx))

            for j in range(idy, self.tRect["y"] + self.tRect["width"]):
                if (self.map[j * self.size_x + idx] == 127):
                    black_count = 0
                    left_and_right_neighbourhood = [[0, -1], [0, 1]]

                    for k in range(2):
                        tmp_idy = j + left_and_right_neighbourhood[k][0]
                        tmp_idx = idx + left_and_right_neighbourhood[k][1]

                        if (tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"] or tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value or self.map[tmp_idy * self.size_x + tmp_idx] == external_corner_value):
                            black_count += 1

                    if (black_count == 1):
                        line.append(Point(j, idx))
                    else:
                        break
                else:
                    break

            if (len(line) > valid_length):
                return True
            else:
                return False

        return False

    def downSearchStatisticalLineLength(self, point, external_corner_value, inner_corner_value, valid_length):
        if (point.x - 1 > self.tRect["y"] and self.map[(point.x - 1) * self.size_x + point.y] == 127):
            idy = point.x - 1
            idx = point.y
            line = []
            line.append(Point(idy, idx))

            for j in range(idy, self.tRect["y"], -1):
                if (self.map[j * self.size_x + idx] == 127):
                    black_count = 0
                    left_and_right_neighbourhood = [[0, -1], [0, 1]]

                    for k in range(2):
                        tmp_idy = j + left_and_right_neighbourhood[k][0]
                        tmp_idx = idx + left_and_right_neighbourhood[k][1]

                        if (tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"] or tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value or self.map[tmp_idy * self.size_x + tmp_idx] == external_corner_value):
                            black_count += 1

                    if (black_count == 1):
                        line.append(Point(j, idx))
                    else:
                        break
                else:
                    break

            if (len(line) > valid_length):
                return True
            else:
                return False
        return False

    def leftSearchStatisticalLineLength(self, point, external_corner_value, inner_corner_value, valid_length):
        if (point.y + 1 < self.tRect["x"] + self.tRect["height"] and self.map[point.x * self.size_x + point.y + 1] == 127):
            idy = point.x
            idx = point.y + 1
            line = []
            line.append(Point(idy, idx))

            for j in range(idx, self.tRect["x"] + self.tRect["height"]):
                if (self.map[idy * self.size_x + j] == 127):
                    black_count = 0
                    up_and_down_neighbourhood = [[-1, 0], [1, 0]]

                    for k in range(2):
                        tmp_idy = idy + up_and_down_neighbourhood[k][0]
                        tmp_idx = j + up_and_down_neighbourhood[k][1]

                        if (tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"] or tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value or self.map[tmp_idy * self.size_x + tmp_idx] == external_corner_value):
                            black_count += 1

                    if (black_count == 1):
                        line.append(Point(idy, j))
                    else:
                        break
                else:
                    break

            if (len(line) > valid_length):
                return True
            else:
                return False
        return False

    def rightSearchStatisticalLineLength(self, point, external_corner_value, inner_corner_value, valid_length):
        if (point.y - 1 > self.tRect["x"] and self.map[point.x * self.size_x + point.y - 1] == 127):
            idy = point.x
            idx = point.y - 1
            line = []
            line.append(Point(idy, idx))

            for j in range(idx, self.tRect["x"], -1):
                if (self.map[idy * self.size_x + j] == 127):
                    black_count = 0
                    up_and_down_neighbourhood = [[-1, 0], [1, 0]]

                    for k in range(2):
                        tmp_idy = idy + up_and_down_neighbourhood[k][0]
                        tmp_idx = j + up_and_down_neighbourhood[k][1]

                        if (tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"] or tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value or self.map[tmp_idy * self.size_x + tmp_idx] == external_corner_value):
                            black_count += 1

                    if (black_count == 1):
                        line.append(Point(idy, j))
                    else:
                        break
                else:
                    break

            if (len(line) > valid_length):
                return True
            else:
                return False

        return False

    def fourNeighbourhoodSearchForExtractCorners(self, point, fill_edges, delete_point, external_corner_value, inner_corner_value, valid_length, is_valid_length) -> tuple[list[Point], list[Point]]:
        delete_point, fill_edges = self.upSearchForExtractCorners(point, fill_edges, delete_point,
                                                                  external_corner_value, inner_corner_value, valid_length, is_valid_length)
        delete_point, fill_edges = self.downSearchForExtractCorners(
            point, fill_edges, delete_point, external_corner_value, inner_corner_value, valid_length, is_valid_length)
        delete_point, fill_edges = self.leftSearchForExtractCorners(
            point, fill_edges, delete_point, external_corner_value, inner_corner_value, valid_length, is_valid_length)
        delete_point, fill_edges = self.rightSearchForExtractCorners(
            point, fill_edges, delete_point, external_corner_value, inner_corner_value, valid_length, is_valid_length)
        return delete_point, fill_edges

    def upSearchForExtractCorners(self, point, fill_edges, delete_point, external_corner_value, inner_corner_value, valid_length, is_valid_length) -> tuple[list[Point], list[Point]]:
        if (point.x + 1 < self.tRect["y"] + self.tRect["width"] and self.map[(point.x + 1) * self.size_x + point.y] == 0):
            idy = point.x + 1
            idx = point.y
            line = []
            line.append(Point(idy, idx))

            for j in range(idy, self.tRect["y"] + self.tRect["width"]):
                if (self.map[j * self.size_x + idx] == 0):
                    black_count = 0
                    left_and_right_neighbourhood = [[0, -1], [0, 1]]

                    for k in range(2):
                        tmp_idy = j + left_and_right_neighbourhood[k][0]
                        tmp_idx = idx + left_and_right_neighbourhood[k][1]

                        if (tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"] or tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == external_corner_value or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            black_count += 1

                    if (black_count == 1):
                        line.append(Point(j, idx))
                    else:
                        break
                else:
                    break

            if (is_valid_length and len(line) > 1):
                line.append(point)
                fill_edges.append(line)

                for i in range(len(line)):
                    _left_and_right_neighbourhood = [[0, -1], [0, 1]]

                    for k in range(2):
                        tmp_idy = line[i].x + \
                            _left_and_right_neighbourhood[k][0]
                        tmp_idx = line[i].y + \
                            _left_and_right_neighbourhood[k][1]

                        if (tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"] or tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            self.map[tmp_idy * self.size_x + tmp_idx] = 127
                            delete_point.append(Point(tmp_idy, tmp_idx))
            elif (len(line) > valid_length):
                line.append(point)
                fill_edges.append(line)

                for i in range(len(line)):
                    _left_and_right_neighbourhood2 = [[0, -1], [0, 1]]

                    for k in range(2):
                        tmp_idy = line[i].x + \
                            _left_and_right_neighbourhood2[k][0]
                        tmp_idx = line[i].y + \
                            _left_and_right_neighbourhood2[k][1]

                        if (tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"] or tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            self.map[tmp_idy * self.size_x + tmp_idx] = 127
                            delete_point.append(Point(tmp_idy, tmp_idx))
            else:
                line = []

        return delete_point, fill_edges

    def downSearchForExtractCorners(self, point, fill_edges, delete_point, external_corner_value, inner_corner_value, valid_length, is_valid_length) -> tuple[list[Point], list[Point]]:
        if (point.x - 1 > self.tRect["y"] and self.map[(point.x - 1) * self.size_x + point.y] == 0):
            idy = point.x - 1
            idx = point.y
            line = []
            line.append(Point(idy, idx))

            for j in range(idy, self.tRect["y"], -1):
                if (self.map[j * self.size_x + idx] == 0):
                    black_count = 0
                    left_and_right_neighbourhood = [[0, -1], [0, 1]]

                    for k in range(2):
                        tmp_idy = j + left_and_right_neighbourhood[k][0]
                        tmp_idx = idx + left_and_right_neighbourhood[k][1]

                        if (tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"] or tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == external_corner_value or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            black_count += 1

                    if (black_count == 1):
                        line.append(Point(j, idx))
                    else:
                        break
                else:
                    break

            if (is_valid_length and len(line) > 1):
                line.append(point)
                fill_edges.append(line)

                for i in range(len(line)):
                    _left_and_right_neighbourhood3 = [[0, -1], [0, 1]]

                    for k in range(2):
                        tmp_idy = line[i].x + \
                            _left_and_right_neighbourhood3[k][0]
                        tmp_idx = line[i].y + \
                            _left_and_right_neighbourhood3[k][1]

                        if (tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"] or tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            self.map[tmp_idy * self.size_x + tmp_idx] = 127
                            delete_point.append(Point(tmp_idy, tmp_idx))
            elif (len(line) > valid_length):
                line.append(point)
                fill_edges.append(line)

                for i in range(len(line)):
                    _left_and_right_neighbourhood4 = [[0, -1], [0, 1]]

                    for k in range(2):
                        tmp_idy = line[i].x + \
                            _left_and_right_neighbourhood4[k][0]
                        tmp_idx = line[i].y + \
                            _left_and_right_neighbourhood4[k][1]

                        if (tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"] or tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            self.map[tmp_idy * self.size_x + tmp_idx] = 127
                            delete_point.append(Point(tmp_idy, tmp_idx))
            else:
                line = []

        return delete_point, fill_edges

    def leftSearchForExtractCorners(self, point, fill_edges, delete_point, external_corner_value, inner_corner_value, valid_length, is_valid_length) -> tuple[list[Point], list[Point]]:
        if (point.y + 1 < self.tRect["x"] + self.tRect["height"] and self.map[point.x * self.size_x + point.y + 1] == 0):
            idy = point.x
            idx = point.y + 1
            line = []
            line.append(Point(idy, idx))

            for j in range(idx, self.tRect["x"] + self.tRect["height"]):
                if (self.map[idy * self.size_x + j] == 0):
                    black_count = 0
                    up_and_down_neighbourhood = [[-1, 0], [1, 0]]

                    for k in range(2):
                        tmp_idy = idy + up_and_down_neighbourhood[k][0]
                        tmp_idx = j + up_and_down_neighbourhood[k][1]

                        if (tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"] or tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == external_corner_value or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            black_count += 1

                    if (black_count == 1):
                        line.append(Point(idy, j))
                    else:
                        break
                else:
                    break

            if (is_valid_length and len(line) > 1):
                line.append(point)
                fill_edges.append(line)

                for i in range(len(line)):
                    _up_and_down_neighbourhood = [[-1, 0], [1, 0]]

                    for k in range(2):
                        tmp_idy = line[i].x + _up_and_down_neighbourhood[k][0]
                        tmp_idx = line[i].y + _up_and_down_neighbourhood[k][1]

                        if (tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"] or tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            self.map[tmp_idy * self.size_x + tmp_idx] = 127
                            delete_point.append(Point(tmp_idy, tmp_idx))
            elif (len(line) > valid_length):
                line.append(point)
                fill_edges.append(line)

                for i in range(len(line)):
                    _up_and_down_neighbourhood2 = [[-1, 0], [1, 0]]

                    for k in range(2):
                        tmp_idy = line[i].x + _up_and_down_neighbourhood2[k][0]
                        tmp_idx = line[i].y + _up_and_down_neighbourhood2[k][1]

                        if (tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"] or tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            self.map[tmp_idy * self.size_x + tmp_idx] = 127
                            delete_point.append(Point(tmp_idy, tmp_idx))
            else:
                line = []

        return delete_point, fill_edges

    def rightSearchForExtractCorners(self, point, fill_edges, delete_point: list[Point], external_corner_value, inner_corner_value, valid_length, is_valid_length) -> tuple[list[Point], list[Point]]:
        if (point.y - 1 > self.tRect["x"] and self.map[point.x * self.size_x + point.y - 1] == 0):
            idy = point.x
            idx = point.y - 1
            line = []
            line.append(Point(idy, idx))

            for j in range(idx, self.tRect["x"], -1):
                if (self.map[idy * self.size_x + j] == 0):
                    black_count = 0
                    up_and_down_neighbourhood = [[-1, 0], [1, 0]]

                    for k in range(2):
                        tmp_idy = idy + up_and_down_neighbourhood[k][0]
                        tmp_idx = j + up_and_down_neighbourhood[k][1]

                        if (tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"] or tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == external_corner_value or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            black_count += 1

                    if (black_count == 1):
                        line.append(Point(idy, j))
                    else:
                        break
                else:
                    break

            if (is_valid_length and len(line) > 1):
                line.append(point)
                fill_edges.append(line)

                for i in range(len(line)):
                    _up_and_down_neighbourhood3 = [[-1, 0], [1, 0]]

                    for k in range(2):
                        tmp_idy = line[i].x + _up_and_down_neighbourhood3[k][0]
                        tmp_idx = line[i].y + _up_and_down_neighbourhood3[k][1]

                        if (tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"] or tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            self.map[tmp_idy * self.size_x + tmp_idx] = 127
                            delete_point.append(Point(tmp_idy, tmp_idx))
            elif (len(line) > valid_length):
                line.append(point)
                fill_edges.append(line)

                for i in range(len(line)):
                    _up_and_down_neighbourhood4 = [[-1, 0], [1, 0]]

                    for k in range(2):
                        tmp_idy = line[i].x + _up_and_down_neighbourhood4[k][0]
                        tmp_idx = line[i].y + _up_and_down_neighbourhood4[k][1]

                        if (tmp_idx < self.tRect["x"] or tmp_idx >= self.tRect["x"] + self.tRect["height"] or tmp_idy < self.tRect["y"] or tmp_idy >= self.tRect["y"] + self.tRect["width"]):
                            continue

                        if (self.map[tmp_idy * self.size_x + tmp_idx] == -128 or self.map[tmp_idy * self.size_x + tmp_idx] == inner_corner_value):
                            self.map[tmp_idy * self.size_x + tmp_idx] = 127
                            delete_point.append(Point(tmp_idy, tmp_idx))
            else:
                line = []

        return delete_point, fill_edges

    def fillEdges(self, contour: list[Point], fill_edges: list[Point], value):
        for i in range(len(fill_edges)):
            edge = fill_edges[i]

            for j in range(len(edge)):
                self.map[edge[j].x * self.size_x + edge[j].y] = value
                contour.append(edge[j])
        return contour

    def fillBlackComponent(self, black_region: list[Point], value):
        for i in range(len(black_region)):
            self.map[black_region[i].x *
                     self.size_x + black_region[i].y] = value

    def fillNonBoundaryNoise2(self, nonBoundaryNoise: list[Point]):
        four_neighbourhood = [[5, 0, 4, 0, 3, 0, 2, 0, 1, 0], [0, 5, 0, 4, 0, 3, 0, 2, 0, 1],
                              [-5, 0, -4, 0, -3, 0, -2, 0, -1, 0], [0, -5, 0, -4, 0, -3, 0, -2, 0, -1]]

        for i in range(len(nonBoundaryNoise)):
            p = nonBoundaryNoise[i]
            self.map[p.x * self.size_x + p.y] = 28

            for neighbourhood in four_neighbourhood:
                tmp_p5 = Point(p.x + neighbourhood[1], p.y + neighbourhood[0])
                tmp_p4 = Point(p.x + neighbourhood[3], p.y + neighbourhood[2])
                tmp_p3 = Point(p.x + neighbourhood[5], p.y + neighbourhood[4])
                tmp_p2 = Point(p.x + neighbourhood[7], p.y + neighbourhood[6])
                tmp_p1 = Point(p.x + neighbourhood[9], p.y + neighbourhood[8])

                if (tmp_p5.x < self.tRect["y"] or tmp_p5.x >= self.tRect["y"] + self.tRect["width"] or tmp_p5.y < self.tRect["x"] or tmp_p5.y >= self.tRect["x"] + self.tRect["height"] or tmp_p4.x < self.tRect["y"] or tmp_p4.x >= self.tRect["y"] + self.tRect["width"] or tmp_p4.y < self.tRect["x"] or tmp_p4.y >= self.tRect["x"] + self.tRect["height"] or tmp_p3.x < self.tRect["y"] or tmp_p3.x >= self.tRect["y"] + self.tRect["width"] or tmp_p3.y < self.tRect["x"] or tmp_p3.y >= self.tRect["x"] + self.tRect["height"] or tmp_p2.x < self.tRect["y"] or tmp_p2.x >= self.tRect["y"] + self.tRect["width"] or tmp_p2.y < self.tRect["x"] or tmp_p2.y >= self.tRect["x"] + self.tRect["height"] or tmp_p3.x < self.tRect["y"] or tmp_p3.x >= self.tRect["y"] + self.tRect["width"] or tmp_p1.y < self.tRect["x"] or tmp_p1.y >= self.tRect["x"] + self.tRect["height"]):
                    continue
                if self.map[tmp_p3.x * self.size_x + tmp_p1.y] == 127:
                    if self.map[tmp_p5.x * self.size_x + tmp_p5.y] == -128 and self.map[tmp_p4.x * self.size_x + tmp_p4.y] == 127 and self.map[tmp_p3.x * self.size_x + tmp_p3.y] == 127 and self.map[tmp_p2.x * self.size_x + tmp_p2.y] == 127:
                        nonBoundaryNoise.append(tmp_p4)
                        nonBoundaryNoise.append(tmp_p3)
                        nonBoundaryNoise.append(tmp_p2)
                        nonBoundaryNoise.append(tmp_p1)
                        break
                    elif self.map[tmp_p4.x * self.size_x + tmp_p4.y] == -128 and self.map[tmp_p3.x * self.size_x + tmp_p3.y] == 127 and self.map[tmp_p2.x * self.size_x + tmp_p2.y] == 127:
                        nonBoundaryNoise.append(tmp_p3)
                        nonBoundaryNoise.append(tmp_p2)
                        nonBoundaryNoise.append(tmp_p1)
                        break
                    elif self.map[tmp_p3.x * self.size_x + tmp_p3.y] == -128 and self.map[tmp_p2.x * self.size_x + tmp_p2.y] == 127:
                        nonBoundaryNoise.append(tmp_p2)
                        nonBoundaryNoise.append(tmp_p1)
                        break
                    elif self.map[tmp_p2.x * self.size_x + tmp_p2.y] == -128:
                        nonBoundaryNoise.append(tmp_p1)
                        break

        for p in nonBoundaryNoise:
            self.map[p.x * self.size_x + p.y] = -128

        return nonBoundaryNoise

    def roomColorByChain(self, roomChain: collections.abc.Iterable[RobotMap.DeviceRoomChainDataInfo]):
        for offset in range(len(self.map)):
            match self.map[offset]:
                case -128: self.map[offset] = -1
                case 127: self.map[offset] = 1

        for room in roomChain:
            self.floodFillSingleChain(room.points, room.roomId)

    def floodFillSingleChain(self, room_points, roomId):
        dst = [roomId] * len(self.map)

        for p in room_points:
            dst[p.y * self.size_x + p.x] = 0

        dst = self.scanLineFloodFill(dst, Point(1, 1), roomId, 0)

        for p in room_points:
            dst[p.y * self.size_x + p.x] = roomId

        for offset in range(len(self.map)):
            if (dst[offset] == roomId and self.map[offset] not in [-1, 0, -9]):
                self.map[offset] = dst[offset]

        if (len(room_points) > 3):
            for p in room_points[1:-1]:
                for di in range(-2, 3):
                    for dj in range(-2, 3):
                        offset = (p.y + di) * self.size_x + p.x + dj
                        if (p.y + di >= 0 and p.y + di < self.size_y and p.x + dj >= 0 and p.x + dj < self.size_x) and self.map[offset] == 1:
                            self.map[offset] = roomId

    def scanLineFloodFill(self, dst: list[int], initial_seed: Point, raw_value, new_value):
        scan_line_seed = [initial_seed]
        tempDst = None

        while (len(scan_line_seed) > 0):
            seed = scan_line_seed[0]
            scan_line_seed.pop(0)
            tempDst, x_left = self.floodFillLine(
                dst, seed, -1, raw_value, new_value)

            tempDst, x_right = self.floodFillLine(
                tempDst, seed, 1, raw_value, new_value)

            scan_line_seed = self.searchLineForNewSeed(
                tempDst, x_left, x_right, seed.y - 1, raw_value, scan_line_seed)
            scan_line_seed = self.searchLineForNewSeed(
                tempDst, x_left, x_right, seed.y + 1, raw_value, scan_line_seed)

        return tempDst

    def floodFillLine(self, dst: list[int], initial_seed, direction, raw_value, new_value) -> tuple[list, int]:
        row = initial_seed.y
        col = initial_seed.x
        boundary = col

        if (direction > 0):
            col += direction

        while (col >= 0 and col < self.size_x):
            if (dst[row * self.size_x + col] == raw_value):
                boundary = col
                dst[row * self.size_x + col] = new_value
                col += direction
            else:
                break

        return dst, boundary

    def searchLineForNewSeed(self, dst: list[int], x_left, x_right, line_row, raw_value, scan_line_seed: list[Point]):
        if (line_row < 0 or line_row > self.size_y - 1):
            return scan_line_seed

        x_right_copy = x_right
        is_find_seed = False

        while (x_right_copy >= x_left):
            if (dst[line_row * self.size_x + x_right_copy] == raw_value):
                if (not is_find_seed):
                    seed = Point(x_right_copy, line_row)
                    scan_line_seed.append(seed)
                    is_find_seed = True
            else:
                is_find_seed = False

            x_right_copy -= 1

        return scan_line_seed

    def fillInternalObstacles(self) -> None:
        if (self.tRect["width"] == 0 and self.tRect["height"] == 0):
            self.findRoiMap()

        contour = self.extractExternalContoursNewStrategy([])

        contour = self.findContourConnectComponent(contour)

        self.fillBlackComponent(contour, 30)
        internal_obstacles = self.findInternalObstacles([])

        self.fillBlackComponent(internal_obstacles, -9)

    def findContourConnectComponent(self, contour: list[Point]):
        eight_neighbourhood = [[-1, 0], [1, 0], [0, -1],
                               [0, 1], [-1, 1], [1, 1], [1, -1], [-1, -1]]

        while (len(contour) != 0):
            seed = contour.pop(0)

            for k in range(8):
                currpoint = Point(
                    seed.x + eight_neighbourhood[k][0], seed.y + eight_neighbourhood[k][1])

                if (self.map[currpoint.x * self.size_x + currpoint.y] != -128 or currpoint.x < self.tRect["y"] or currpoint.x >= self.tRect["y"] + self.tRect["width"] or currpoint.y < self.tRect["x"] or currpoint.y >= self.tRect["x"] + self.tRect["height"]):
                    continue

                self.map[currpoint.x * self.size_x + currpoint.y] = 30
                contour.append(currpoint)

        return contour

    def findInternalObstacles(self, point_deque: list[Point]):
        for idy in range(self.tRect["y"], self.tRect["y"] + self.tRect["width"]):
            for idx in range(self.tRect["x"], self.tRect["x"] + self.tRect["height"]):
                if (self.map[idy * self.size_x + idx] == -128):
                    point_deque.append(Point(idy, idx))
        return point_deque
