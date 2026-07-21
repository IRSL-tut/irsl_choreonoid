from cnoid.IRSLCoords import coordinates

class cascadedCoords(coordinates):
    """class cascadedCoords(cnoid.IRSLCoords.coordinates)
    """
    def __init__(self, *args):
        """
        Args:
            \*args (list) : pass to coordinates.__init__(self, *args)
        """
        super().__init__(*args)
        ##
        self.children = []
        self._resetParent()

    def assoc(self, child, coords=None):
        """Associate coordinates as a child
        Fix current transformation from self to child.
        World coords of self and child will be not changed.

        Args:
            child ( irsl_choreonoid.irsl_draw_object.cascadedCoords ) : child coordinates
            coords ( cnoid.Body.Link or cnoid.Util.SgPosTransform) : Coordinates on parent

        Example:
        >>> cas1,assoc(cas2)
        >>> abody.assoc(cas1, abody.link('A_LINK'))
        """
        if child._parent is None:
            child._setParent(self, coords=coords)
            self.children.append(child)

    def dissoc(self, child):
        """Finish association with child coordinates

        Args:
            child ( irsl_choreonoid.irsl_draw_object.cascadedCoords ) :
        """
        if self is child._parent:
            child._resetParent()
            if child in self.children:
                self.children.remove(child)

    def dissocFromParent(self):
        """Finish association with parent coordinates
        """
        if self._parent is not None:
            self._parent.dissoc(self)

    def clearChildren(self):
        """Clear all children
        """
        for c in self.children:
            self.dissoc(c)
        self.children = []

    @property
    def descendants(self):
        """Getting all children
        """
        return self.children

    @property
    def ancestor(self):
        """
        """
        return self._parent

    @property
    def fromParent(self):
        """
        """
        return self._from_parent

    @property
    def parentCoords(self):
        """
        """
        return self._parent_coords

    def isChild(self, coords):
        """Query: Is it a child ?
        """
        return coords in self.children

    def isParent(self, coords):
        """Query: Is it a parent ?
        """
        return coords is self._parent

    def hasChild(self):
        """Query: Does it have child ?
        """
        return len(self.children) > 0

    def setFromParent(self, coords, update=True):
        """
        """
        if self._parent is not None:
            self._from_parent = coords
            self._update_from_parent(update=False)
            self._updateChildren(update=False)

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

    def _updateParent(self): ## updating self._from_parent
        if self._parent is not None:
            self._setParent(self._parent, self._parent_coords)

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

    def updateTarget(self, update=True):
        """Updating self.target and call callback_function

        Args:
            None

        """
        self._updateParent()
        self._updateChildren(update=False)

    ##
    ## override methods : coordinates
    ##
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
            axis (numpy.array) : 1x3 vector, axis to rotate around
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
            axis (numpy.array) : 1x3 vector, axis to rotate around
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
        return 'cas-coords {} : '.format(type(self)) + super().__repr__()
