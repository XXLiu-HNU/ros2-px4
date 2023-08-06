# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uav_interfaces:srv/UavControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UavControl_Request(type):
    """Metaclass of message 'UavControl_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uav_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uav_interfaces.srv.UavControl_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__uav_control__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__uav_control__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__uav_control__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__uav_control__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__uav_control__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UavControl_Request(metaclass=Metaclass_UavControl_Request):
    """Message class 'UavControl_Request'."""

    __slots__ = [
        '_takeoff_land',
        '_x_deta',
        '_y_deta',
        '_z_deta',
    ]

    _fields_and_field_types = {
        'takeoff_land': 'int32',
        'x_deta': 'int32',
        'y_deta': 'int32',
        'z_deta': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.takeoff_land = kwargs.get('takeoff_land', int())
        self.x_deta = kwargs.get('x_deta', int())
        self.y_deta = kwargs.get('y_deta', int())
        self.z_deta = kwargs.get('z_deta', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.takeoff_land != other.takeoff_land:
            return False
        if self.x_deta != other.x_deta:
            return False
        if self.y_deta != other.y_deta:
            return False
        if self.z_deta != other.z_deta:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def takeoff_land(self):
        """Message field 'takeoff_land'."""
        return self._takeoff_land

    @takeoff_land.setter
    def takeoff_land(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'takeoff_land' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'takeoff_land' field must be an integer in [-2147483648, 2147483647]"
        self._takeoff_land = value

    @property
    def x_deta(self):
        """Message field 'x_deta'."""
        return self._x_deta

    @x_deta.setter
    def x_deta(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'x_deta' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'x_deta' field must be an integer in [-2147483648, 2147483647]"
        self._x_deta = value

    @property
    def y_deta(self):
        """Message field 'y_deta'."""
        return self._y_deta

    @y_deta.setter
    def y_deta(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'y_deta' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'y_deta' field must be an integer in [-2147483648, 2147483647]"
        self._y_deta = value

    @property
    def z_deta(self):
        """Message field 'z_deta'."""
        return self._z_deta

    @z_deta.setter
    def z_deta(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'z_deta' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'z_deta' field must be an integer in [-2147483648, 2147483647]"
        self._z_deta = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_UavControl_Response(type):
    """Metaclass of message 'UavControl_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uav_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uav_interfaces.srv.UavControl_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__uav_control__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__uav_control__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__uav_control__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__uav_control__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__uav_control__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UavControl_Response(metaclass=Metaclass_UavControl_Response):
    """Message class 'UavControl_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'success' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'success' field must be an integer in [-2147483648, 2147483647]"
        self._success = value


class Metaclass_UavControl(type):
    """Metaclass of service 'UavControl'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uav_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uav_interfaces.srv.UavControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__uav_control

            from uav_interfaces.srv import _uav_control
            if _uav_control.Metaclass_UavControl_Request._TYPE_SUPPORT is None:
                _uav_control.Metaclass_UavControl_Request.__import_type_support__()
            if _uav_control.Metaclass_UavControl_Response._TYPE_SUPPORT is None:
                _uav_control.Metaclass_UavControl_Response.__import_type_support__()


class UavControl(metaclass=Metaclass_UavControl):
    from uav_interfaces.srv._uav_control import UavControl_Request as Request
    from uav_interfaces.srv._uav_control import UavControl_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
