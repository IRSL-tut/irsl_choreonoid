## do not use cnoid.Base
from cnoid.IRSLCoords import coordinates
import cnoid.Util as cutil
# from cnoid.Util import SgPosTransform
# from cnoid.DrawInterface import GeneralDrawInterface as GDI

class coordsWrapper(coordinates):
    """class coordsWrapper(cnoid.IRSLCoords.coordinates)

This class aims to wrap a cnoid Object which have position (SgPosTransform, Link, etc.)

You can access to this object with methods in cnoid.IRSLCoords.coordinates,

Some mthods (newcoords, translate, rotate, transform) to update itself are overrided to add calling callback_function.

Then, you can run some process when the position of the target is updated.
    """
    def __init__(self, target, init_coords=None, update_callback=None, kinematics_callback=None, original_object=None, scalable=False):
        """
        Args:
            target (object) : wrapped target which have property 'T' for setting cnoidPosition
            init_coords (cnoid.IRSLCoords.coordinates, optional) : coordinates of this instance
            update_callback (function(), optional) : callback function which is called when target is updated (including drawing)
            kinematics_callback (function(), optional) : callback function which is called when target is updated (only updating kinematics)
            scalable (boolean, default=False) : call setScalable() within constructor
        """
        super().__init__()
        self.__target = target
        self._original_object=original_object
        ##if hasattr(target, 'coords'):
        ##    target.coords = self
        ##else:
        ##    setattr(target, 'coords', self)

        self.__update_callback = update_callback
        self.__kinematics_callback = kinematics_callback
        if init_coords is not None:
            self.newcoords(init_coords)
        else:
            self.cnoidPosition =  self.__target.T

        if scalable:
            self.setScalable()
        self.children = []
        self._resetParent()
### cascaded-coordinates
    def assoc(self, child, coords=None):
        if child._parent is None:
            child._setParent(self, coords=coords)
            self.children.append(child)

    def dissoc(self, child):
        if self is child.parent:
            child._resetParent()
            if child in self.children:
                self.children.remove(child)

    def dissocFromParent(self):
        if self._parent is not None:
            self._parent.dissoc(self)

    def clearChildren(self):
        for c in self.children:
            self.dissoc(c)
        self.children = []

    def descendants(self):
        return self.children

    def isChild(self, coords):
        return coords in self.children

    def isParent(self, coords):
        return coords is self._parent

    def hasChild(self):
        return len(self.children) > 0

    def _resetParent(self):
        self._parent = None
        self._from_parent = None
        self._parent_coords = None ## for using Link or SgPosTransform

    def _setParent(self, parent, coords=None):
        self._parent = parent
        self._parent_coords = coords
        if coords is not None:
            if hasattr(coords, 'T'):
                self._from_parent = coordinates(coords.T).transformation(self)
            else:
                self._from_parent = coords.transformation(self)
        else:
            self._from_parent = parent.transformation(self)

    def _updateChildren(self, update=False):
        for c in self.children:
            c._update_from_parent(update=update)

    def _update_from_parent(self, update=False):
        if self._parent_coords is None:
            super().newcoords(self._parent)
        else:
            if hasattr(self._parent_coords, 'T'):
                super().newcoords(coordinates(self._parent_coords.T))
            else:
                super().newcoords(self._parent_coords)
        super().transform(self._from_parent)
        self.updateTarget(update=update)
###
    def setScalable(self):
        """Enabling to use methods, setScale and setTurnedOn

        """
        if self.__target.numChildren == 1:
            scl = cutil.SgScaleTransform()
            swt = cutil.SgSwitchableGroup()
            self._switch_ = swt
            self._scale_  = scl
            chld = self.__target.getChild(0)
            self.__target.clearChildren()
            scl.addChild(chld)
            swt.addChild(scl)
            self.__target.addChild(swt)
        else:
            trs = cutil.SgPosTransform()
            trs.T = self.__target.T
            self.__target.T = coordinates().cnoidPosition
            org = self.__target
            trs.addChild(org)
            self.__target = trs
            self.setScalable()

    #def __del__(self):
    #    print('destruct: coordsWrapper')

    def updateTarget(self, update=True):
        """Updating self.target and call callback_function

        Args:
            None

        """
        self.__target.T = self.cnoidPosition
        if callable(self.__kinematics_callback):
            self.__kinematics_callback()
        self._updateChildren(update=False)
        if update and callable(self.__update_callback):
            self.__update_callback()

    def revert(self): ##
        self.cnoidPosition =  self.__target.T

    def setUpdateCallback(self, func):
        """Setting callback function for updating drawings

        Args:
            func (function()) : callback function which is called when target is updated

        """
        if callable(func):
            self.__update_callback = func

    def setKinematicsCallback(self, func):
        """Setting callback function for updating kinematics

        Args:
            func (function()) : callback function which is called when target is updated

        """
        if callable(func):
            self.__kinematics_callback = func

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

    def setScale(self, float_or_vec, update=False):
        """Setting scale of drawn objects

        Args:
            float_or_vec (float or [float] ) : Parameter of scale
            update (boolean, default=False) : Update drawn object in screen
        """
        self._scale_.setScale(float_or_vec)
        if update:
            self.updateTarget()

    def setTurnedOn(self, on=True, update=False):
        """Switching On/Off of drawn objects

        Args:
            on (boolean, default=False) : Switch to draw this object
            update (boolean, default=False) : Update drawn object in screen
        """
        self._switch_.setTurnedOn(on, False)
        if update:
            self.updateTarget()
