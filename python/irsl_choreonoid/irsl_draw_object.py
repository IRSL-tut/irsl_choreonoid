## do not use cnoid.Base
from cnoid.IRSLCoords import coordinates
# from cnoid.Util import SgPosTransform
# from cnoid.DrawInterface import GeneralDrawInterface as GDI

class coordsWrapper(coordinates):
    """class coordsWrapper(cnoid.IRSLCoords.coordinates)

This class aims to wrap a cnoid Object which have position (SgPosTransform, Link, etc.)

You can access to this object with methods in cnoid.IRSLCoords.coordinates,

Some mthods (newcoords, translate, rotate, transform) to update itself are overrided to add calling callback_function.

Then, you can run some process when the position of the target is updated.
    """
    def __init__(self, target, init_coords=None, update_callback=None, original_object=None):
        """
        Args:
            target (object) : wrapped target which have property 'T' for setting cnoidPosition
            init_coords (cnoid.IRSLCoords.coordinates, optional) : coordinates of this instance
            update_callback (function(), optional) : callback function which is called when target is updated
        """
        super().__init__()
        self.__target = target
        self._original_object=original_object
        ##if hasattr(target, 'coords'):
        ##    target.coords = self
        ##else:
        ##    setattr(target, 'coords', self)

        self.__update_callback = update_callback

        if init_coords is not None:
            self.newcoords(init_coords)
        else:
            self.cnoidPosition =  self.__target.T

    #def __del__(self):
    #    print('destruct: coordsWrapper')

    def updateTarget(self):
        """Updating self.target and call callback_function

        Args:
            None

        """
        self.__target.T = self.cnoidPosition
        if callable(self.__update_callback):
            self.__update_callback()

    def revert(self): ##
        self.cnoidPosition =  self.__target.T

    def setUpdateCallback(self, func):
        """Setting callback function

        Args:
            func (function()) : callback function which is called when target is updated

        """
        if callable(func):
            self.__update_callback = func

    def newcoords(self, cds):
        """Wrapped method of newcoords in cnoid.IRSLCoords.coordinates

        Args:
            cds (cnoid.IRSLCoords.coordinates) : replace pos and rot in self with cds

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method
        """
        super().newcoords(cds)
        self.updateTarget()
        return self

    def translate(self, pos, wrt = None):
        """Wrapped method of translate in cnoid.IRSLCoords.coordinates

        Args:
            pos (numpy.array) : 1x3 vector, translation vector
            wrt (cnoid.IRSLCoords.coordinates.wrt or cnoid.IRSLCoords.coordinates, optional) : Reference coordinates applying this method

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method
        """
        if wrt is None:
            super().translate(pos)
        else:
            super().translate(pos, wrt)
        self.updateTarget()
        return self

    def locate(self, pos, wrt = None):
        """Wrapped method of locate in cnoid.IRSLCoords.coordinates

        Args:
            pos (numpy.array) : 1x3 vector, translation vector
            wrt (cnoid.IRSLCoords.coordinates.wrt or cnoid.IRSLCoords.coordinates, optional) : Reference coordinates applying this method

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method
        """
        if wrt is None:
            super().locate(pos)
        else:
            super().locate(pos, wrt)
        self.updateTarget()
        return self

    def rotate(self, ang, axis, wrt = None):
        """Wrapped method of rotate in cnoid.IRSLCoords.coordinates

        Args:
            ang (float) : angle to rotate [radian]
            aixs (numpy.array) : 1x3 vector, axis to rotate around
            wrt (cnoid.IRSLCoords.coordinates.wrt or cnoid.IRSLCoords.coordinates, optional) : Reference coordinates applying this method

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method

        """
        if wrt is None:
            super().rotate(ang, axis)
        else:
            super().rotate(ang, axis, wrt)
        self.updateTarget()
        return self

    def orient(self, ang, axis, wrt = None):
        """Wrapped method of orient in cnoid.IRSLCoords.coordinates

        Args:
            ang (float) : angle to rotate [radian]
            aixs (numpy.array) : 1x3 vector, axis to rotate around
            wrt (cnoid.IRSLCoords.coordinates.wrt or cnoid.IRSLCoords.coordinates, optional) : Reference coordinates applying this method

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method

        """
        if wrt is None:
            super().orient(ang, axis)
        else:
            super().orient(ang, axis, wrt)
        self.updateTarget()
        return self

    def rotate_with_matrix(self, rot, wrt = None):
        """Wrapped method of rotate_with_matrix in cnoid.IRSLCoords.coordinates

        Args:
            rot (numpy.array) : 3x3 matrix, matrix applied to rotate
            wrt (cnoid.IRSLCoords.coordinates.wrt or cnoid.IRSLCoords.coordinates, optional) : Reference coordinates applying this method

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method

        """
        if wrt is None:
            super().rotate_with_matrix(rot)
        else:
            super().rotate_with_matrix(rot, wrt)
        self.updateTarget()
        return self

    def orient_with_matrix(self, rot, wrt = None):
        """Wrapped method of orient_with_matrix in cnoid.IRSLCoords.coordinates

        Args:
        rot (numpy.array) : 3x3 matrix, matrix applied to rotate
            wrt (cnoid.IRSLCoords.coordinates.wrt or cnoid.IRSLCoords.coordinates, optional) : Reference coordinates applying this method

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method

        """
        if wrt is None:
            super().orient_with_matrix(rot)
        else:
            super().orient_with_matrix(rot, wrt)
        self.updateTarget()
        return self

    def transform(self, trs, wrt = None):
        """Wrapped method of transform in cnoid.IRSLCoords.coordinates

        Args:
            trs (cnoid.IRSLCoords.coordinates) : Transformation to be applied
            wrt (cnoid.IRSLCoords.coordinates.wrt or cnoid.IRSLCoords.coordinates, optional) : Reference coordinates applying this method

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method

        """
        if wrt is None:
            super().transform(trs)
        else:
            super().transform(trs, wrt)
        self.updateTarget()
        return self

    def move_to(self, trs, wrt = None):
        """Wrapped method of move_to in cnoid.IRSLCoords.coordinates

        Args:
            trs (cnoid.IRSLCoords.coordinates) : Transformation to be applied
            wrt (cnoid.IRSLCoords.coordinates.wrt or cnoid.IRSLCoords.coordinates, optional) : Reference coordinates applying this method

        Returns:
            cnoid.IRSLCoords.coordinates : identical instance which was called with this method

        """
        if wrt is None:
            super().move_to(trs)
        else:
            super().move_to(trs, wrt)
        self.updateTarget()
        return self

    def __repr__(self):
        if self is self.target:
            return 'Wrap {} : '.format(type(self)) + super().__repr__()
        else:
            return 'Wrap: ' + super().__repr__() + ' | ' + self.target.__repr__()

    @property
    def target(self):
        """Wrapped target of this instance, which was manipulated by methods of this class

        Returns:
            object : instance which have method 'T' to set cnoidPosition (4x4 matrix)

        """
        return self.__target

    @property
    def object(self):
        """Wrapped object of this instance, which was manipulated by methods of this class

        Returns:
            object : Utility slot set while initializing

        Note:
            target is a transformed target such as cnoid.Util.SgPosTransform
            object is a utility slot for handling a child of the target (Shape, etc...)

        """
        return self._original_object
