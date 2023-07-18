import cnoid.DrawInterface as di
import numpy as np

###
class DrawCoords(object):
##    """deprecated, use DrawCoordsList"""
    def __init__(self, color=None, width=None):
        self.width = width
        if color is None:
            self.XLine = di.DrawInterface(np.array([1, 0, 0]))
            self.YLine = di.DrawInterface(np.array([0, 1, 0]))
            self.ZLine = di.DrawInterface(np.array([0, 0, 1]))
        elif type(color) is np.ndarray:
            self.XLine = di.DrawInterface(color)
            self.YLine = di.DrawInterface(color)
            self.ZLine = di.DrawInterface(color)
        elif (type(color) is list) and ( (type(color[0]) is int) or (type(color[0]) is float) ):
            self.XLine = di.DrawInterface(np.array(color))
            self.YLine = di.DrawInterface(np.array(color))
            self.ZLine = di.DrawInterface(np.array(color))
        elif (type(color) is list) and (type(color[0]) is np.ndarray):
            self.XLine = di.DrawInterface(color[0])
            self.YLine = di.DrawInterface(color[1])
            self.ZLine = di.DrawInterface(color[2])
        elif (type(color) is list) and (type(color[0]) is list):
            self.XLine = di.DrawInterface(np.array(color[0]))
            self.YLine = di.DrawInterface(np.array(color[1]))
            self.ZLine = di.DrawInterface(np.array(color[2]))
        else:
            self.XLine = di.DrawInterface(np.array([1, 0, 0]))
            self.YLine = di.DrawInterface(np.array([0, 1, 0]))
            self.ZLine = di.DrawInterface(np.array([0, 0, 1]))

    def __del__(self):
        # print('del: {:X}'.format(id(self)))
        self.hide()
        self.XLine = None
        self.YLine = None
        self.ZLine = None
    def __repr__(self):
        return '<DrawCoords {:X}({:X},{:X},{:X})>'.format(id(self), id(self.XLine), id(self.YLine), id(self.ZLine))

    def hide(self, flush = True):
        self.XLine.hide(False)
        self.YLine.hide(False)
        self.ZLine.hide(False)
        if flush:
            di.flush()
    def show(self, flush = True):
        self.XLine.show(False)
        self.YLine.show(False)
        self.ZLine.show(False)
        if flush:
            di.flush()
    def hide_and_show(self, flush = True):
        self.XLine.hide_and_show()
        self.YLine.hide_and_show()
        self.ZLine.hide_and_show()
        if flush:
            di.flush()
    def draw_simple(self, coords, length=0.1, flush=True):
        # self.hide(flush=flush)
        if not self.width is None:
            self.XLine.setLineWidth(self.width)
            self.YLine.setLineWidth(self.width)
            self.ZLine.setLineWidth(self.width)
        self.XLine.drawAxis(coords, 0, length)
        self.YLine.drawAxis(coords, 1, length)
        self.ZLine.drawAxis(coords, 2, length)
        self.show(flush=flush)
    def draw(self, coords, length = 0.1, axis_size = 0.02, flush=True):
        self.hide(flush=flush)
        rot = iu.Position_rotation(coords)
        ax_x = length * rot[:3, 0]
        ax_y = length * rot[:3, 1]
        ax_z = length * rot[:3, 2]
        ax_vec = length * iu.normalizeVector(rot.dot(np.array([1, 1, 1])))
        pp = iu.Position_translation(coords)
        if not self.width is None:
            self.XLine.setLineWidth(self.width)
            self.YLine.setLineWidth(self.width)
            self.ZLine.setLineWidth(self.width)
        self.XLine.drawArrow(pp, pp + ax_x, axis_size, ax_vec, 15)
        self.YLine.drawArrow(pp, pp + ax_y, axis_size, ax_vec, 15)
        self.ZLine.drawArrow(pp, pp + ax_z, axis_size, ax_vec, 15)
        self.show(flush=flush)

# class DrawCoordsList(object):
#     def __init__(self):
#         self.coords_list = []
#         self.count = 0
#     def flush(self):
#         di.flush()
#     def hide(self, start=0, length=0):
#         if length == 0:
#             end = None
#         else:
#             end = start + length
#         for l in self.coords_list[start:end]:
#             l.hide(flush=False)
#         di.flush()
#     def show(self, start=0, length=0):
#         if length == 0:
#             end = None
#         else:
#             end = start + length
#         for l in self.coords_list[start:end]:
#             l.show(flush=False)
#         di.flush()
#     def clear(self):
#         for l in self.coords_list:
#             l.hide(flush=False)
#         di.flush()
#         self.coords_list = []
#         self.count = 0
#     def generatePointFunction(self, length=0.1, maxlength=0, flush=True):
#         def closure_func__(lst, **kwargs):
#             pos = np.array(lst[0][1:])
#             cd = DrawCoords()
#             cds_ = iu.coordinates(pos)
#             cd.draw_simple(cds_, length=length, flush=flush)
#             self.coords_list.append(cd)
#             self.count += 1
#         return closure_func__
#     def generateCoordsFunction(self, length=0.1, maxlength=0, flush=True):
#         def closure_func__(lst, **kwargs):
#             lst = lst[0]
#             pos = np.array(lst[1:4])
#             rpy = np.array(lst[4:7])
#             cd = DrawCoords()
#             cds_ = iu.coordinates(pos)
#             cds_.setRPY(rpy)
#             cd.draw_simple(cds_, length=length, flush=flush)
#             self.coords_list.append(cd)
#             self.count += 1
#        return closure_func__

class DrawCoordsList(object):
    """DrawCoordsList(class)
    """
    def __init__(self, x_color=np.array([1,0,0]), y_color=np.array([0,1,0]), z_color=np.array([0,0,1]), length=0.1, width=None):
        """DrawCoordsList(initializer)

        Args:

        Returns:

        """
        self.x_color = x_color
        self.y_color = y_color
        self.z_color = z_color
        self.length = length
        self.reset()
        if width is not None:
            self.__interface.setLineWidth(width)

    def __del__(self):
        self.hide()
        self.__interface = None

    def setOrigin(self, coords):
        """setOrigin(self, coords):

        Args:

        Returns:

        """
        self.__interface.setOrigin(coords)

    def getOrigin(self):
        """getOrigin(self):

        Args:

        Returns:

        """
        return self.__interface.getOrigin()

    def setLineWidth(self, _width):
        """setLineWidth(self, _width):

        Args:

        Returns:

        """
        self.__interface.setLineWidth(_width)

    def reset(self):
        """reset(self):

        Args:

        Returns:

        """
        self.__interface = di.DrawInterface(self.x_color)
        self.x_color_index = 0
        self.y_color_index = self.__interface.addColor(self.y_color)
        self.z_color_index = self.__interface.addColor(self.z_color)
        self.count = 0

    def flush(self):
        """flush(self):

        Args:

        Returns:

        """
        di.flush()

    def hide(self, start=0, length=0):
        """hide(self, start=0, length=0):

        Args:

        Returns:

        """
        self.__interface.hide(False)
        di.flush()

    def show(self, start=0, length=0):
        """show(self, start=0, length=0):

        Args:

        Returns:

        """
        self.__interface.show(False)
        di.flush()

    def clear(self):
        """clear(self):

        Args:

        Returns:

        """
        self.__interface.hide(False)
        di.flush()
        self.reset()

    def addCoords(self, coords, flush=False):
        """addCoords(self, coords, flush=False):

        Args:

        Returns:

        """
        if flush:
            self.__interface.hide(False)
        self.__interface.addAxis3(coords, self.length, self.x_color_index, self.y_color_index, self.z_color_index)
        if flush:
            self.__interface.show(True)
            di.flush()
        self.count += 1

    def addCross(self, coords, flush=False):
        """addCross(self, coords, flush=False):

        Args:

        Returns:

        """
        if flush:
            self.__interface.hide(False)
        self.__interface.addBDAxis3(coords, self.length, self.x_color_index, self.y_color_index, self.z_color_index)
        if flush:
            self.__interface.show(True)
            di.flush()
        self.count += 1

    def generatePointFunction(self, length=0.1, maxlength=0, index=0, flush=True):
        """generatePointFunction(self, length=0.1, maxlength=0, index=0, flush=True):

        Args:

        Returns:

        """
        def closure_func__(lst, **kwargs):
            pos = np.array(lst[index][1:])
            cds_ = iu.coordinates(pos)
            self.addCross(cds_,flush=flush)
        return closure_func__

    def generateCoordsFunction(self, length=0.1, maxlength=0, index=0, flush=True):
        """generateCoordsFunction(self, length=0.1, maxlength=0, index=0, flush=True):

        Args:

        Returns:

        """
        def closure_func__(lst, **kwargs):
            lst = lst[index]
            pos = np.array(lst[1:4])
            rpy = np.array(lst[4:7])
            cds_ = iu.coordinates(pos)
            cds_.setRPY(rpy)
            self.addCoords(cds_,flush=flush)
        return closure_func__
