'''
Contains implementation of the message base class and mutable and immutable types.
The last 2 are basically deprecated and might be removed. 

Considered operations to be protected:
    - Assignment;
        - Attributes should not be re-assigned with invalid types
    - Access (getter);
        - A reference to a mutable object should not inadvertently be exposed
    - Initialization.
        - Attributes should not be initialized with invalid types
    - Dynamic creation of attributes
        - Attributes should not be created/added to the module classes (Even if this is not
        dangerous per se, this might induce programming errors)
'''

import copy
from enum import IntEnum, IntFlag
from collections import namedtuple
import time
from typing import Optional, Any

import pyimclsts.core as core
from . import enumerations as imc_enums
from . import bitfields as imc_bitf

# "Global" definitions
header_data = namedtuple('header_data', ['sync', 'mgid', 'size', 'timestamp', 'src', 'src_ent', 'dst', 'dst_ent'])
MessageAttributes = namedtuple('MessageAttributes', ['abbrev', 'usedby', 'stable', 'id', 'category', 'source', 'fields', 'description', 'name', 'flags'])

# "Global" variables
_sync_number = 0xfe54
_default_src = (0x4000 | (core.get_initial_IP() & 0x1FFF)) + 1

# "Re-exporting" from core
IMC_message = core.IMC_message

imc_types = {'int8_t': int, 'uint8_t': int, 'int16_t': int, 'uint16_t': int, 'int32_t': int, 'uint32_t': int, 'int64_t': int, 'fp32_t': float, 'fp64_t': float, 'rawdata': bytes, 'plaintext': str, 'message': IMC_message, 'message-list': list}

class base_message(IMC_message):
    
    __slots__ = ['_header', '_footer', 'Attributes']
    # This is the definition of the print message
    def __str__(self) -> str:
        output = ['Message \'' + self.Attributes.name + '\':', 'Fields:']
        for field in self.Attributes.fields:
            value = getattr(self, field)
            if value is not None:
                if type(value) == str:
                    value = '\'' + value + '\''
                
                #Check whether it is an Enumerated type
                elif getattr(getattr(type(self), field), '_field_def').get('unit', None) in ['Enumerated', 'Bitfield']:
                    # check whether the Enum was "validated" by the descriptor
                    if isinstance(value, IntEnum) or isinstance(value, IntFlag):
                        enum_def = getattr(getattr(type(self), field), '_field_def').get('enum-def', None) if getattr(getattr(type(self), field), '_field_def').get('enum-def', None) \
                            else getattr(getattr(type(self), field), '_field_def').get('bitfield-def', None)
                        
                        # check whether it is local or global
                        if not enum_def:
                            value = self.Attributes.abbrev + '.' + str(value)
                        else:
                            value = str(value)
                    else:
                        value = str(value)

                elif isinstance(value, IMC_message):
                    value = ('\n' + str(value)).replace('\n', '\n    ')
                elif type(value) == list:
                    value = ('\n[\n' + '\n'.join([str(v) for v in value]) + '\n]').replace('\n', '\n    ')
                else:
                    value = str(value)
            else:
                value = 'None'

            output.append('  - ' + field + " = " + value)
        output = '\n'.join(output)
        if hasattr(self, '_header'):
            output = repr(self._header) + '\n' + output
        return output

    def __repr__(self) -> str:
        arguments = []
        for field in self.Attributes.fields:
            value = getattr(self, field)            
            if value is not None:
                #Check whether it is an Enumerated type
                if getattr(getattr(type(self), field), '_field_def').get('unit', None) in ['Enumerated', 'Bitfield']:
                    # check whether the Enum was "validated" by the descriptor
                    if isinstance(value, IntEnum) or isinstance(value, IntFlag):
                        enum_def = getattr(getattr(type(self), field), '_field_def').get('enum-def', None) \
                            if getattr(getattr(type(self), field), '_field_def').get('enum-def', None) \
                            else getattr(getattr(type(self), field), '_field_def').get('bitfield-def', None)

                        # check whether it is local or global
                        if enum_def:
                            value = str(value)
                        else:
                            value = self.Attributes.abbrev + '.' + str(value)
                    else:
                        value = repr(value)
                else:
                    value = repr(value)
            else:
                value = 'None'

            arguments.append(field + " = " + value)
        arguments = ', '.join(arguments)
        output = self.Attributes.abbrev + '({})'.format(arguments)
        return output

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, base_message):
            # if both of them have already defined a '_header'
            if hasattr(self, '_header') and hasattr(__o, '_header'):
                if self._header != __o._header:
                    return False
            # if any of them does not have '_header' -> happens when a message is 
            # inside another, I'll skip and check the contents
            
            if hasattr(self, '_header') and hasattr(__o, '_header'):
                if self._header != __o._header:
                    return False
            fields = self.Attributes.fields
            if any([getattr(self, field) != getattr(__o, field) for field in fields]):
                return False
            return True
        return False

    def _pack_fields(self, *, serial_functions : dict) -> bytes:
        # Check if any field is empty (None) and not type 'message' (checked through the descriptor)
        if any([getattr(self, '_' + field) is None for field in self.Attributes.fields if getattr(getattr(type(self), field), '_field_def').get('type', None) != 'message']):
            raise ValueError('Cannot serialize a message that contains an empty (NoneType) field that is not a message.')
        
        serial_functions = serial_functions

        serialized_fields = []
        for field in self.Attributes.fields:
            # Access variable information through the descriptor
            datatype = getattr(getattr(type(self), field), '_field_def').get('type', None)
            
            # check if it is a "NULL" message
            if datatype == 'message' and getattr(self, '_' + field) is None:
                serialized_fields.append(serial_functions['uint16_t'](65535))
            else:
                serialized_fields.append(serial_functions[datatype](getattr(self, '_' + field)))
        
        return b''.join(serialized_fields)

    def _pack_header(self, *, serial_functions : dict, size : int, src : Optional[int] = None, src_ent : Optional[int] = None, dst : Optional[int] = None, dst_ent : Optional[int] = None) -> bytes:
        '''Gathers necessary builds the header, stores it in the private variable '_header' and returns the bit string.
        
        The tricky part is: Are the header fields fixed or not, that is, they should be hardcoded or not?
        There may be mutability on their: 1. existence; 2, name; 3. order; 4. type (and size).
        
        We will assume that only 4. can change.
            - As a result a namedTuple is globally defined.'''

        mgid = self.Attributes.id

        # If None or a "default" value, overwrite
        if not hasattr(self, '_header'):
            _timestamp = time.time()
            _src = src if src is not None else _default_src
            _src_ent = src_ent if src_ent is not None else 0xFF
            _dst = dst if dst is not None else 0xFFFF
            _dst_ent = dst_ent if dst_ent is not None else 0xFF
        else:
            _timestamp = self._header.timestamp
            _src = src if src is not None else self._header.src
            _src_ent = src_ent if src_ent is not None else self._header.src_ent
            _dst = dst if dst is not None else self._header.dst
            _dst_ent = dst_ent if dst_ent is not None else self._header.dst_ent
        
        header_fields_values = header_data(sync=_sync_number, mgid=mgid, size=size, timestamp=_timestamp, src=_src, src_ent=_src_ent, dst=_dst, dst_ent=_dst_ent)
        
        self._header = header_fields_values

        return serial_functions['header'](*self._header)
    
    def pack(self, *, is_field_message : bool = False, is_big_endian : bool = True, src : Optional[int] = None, src_ent : Optional[int] = None, 
                        dst : Optional[int] = None, dst_ent : Optional[int] = None) -> bytes:
        '''Serialize function that optionally overwrites the header, if parameters are provided.'''
        
        serial_functions = core.pack_functions_big if is_big_endian else core.pack_functions_little
        
        s_fields = self._pack_fields(serial_functions=serial_functions)
        
        if not is_field_message:
        
            s_header = self._pack_header(serial_functions=serial_functions, size=(len(s_fields)), src=src, src_ent=src_ent, dst=dst, dst_ent=dst_ent)
            s_message = s_header + s_fields
            
            # footer:
            '''Calculates CRC-16 IBM of a bit string'''
            self._footer = core.CRC16IMB(s_message)
            s_message = s_message + serial_functions['uint16_t'](self._footer)

            return s_message
        return serial_functions['uint16_t'](self.Attributes.id) + s_fields

    def get_timestamp(self) -> Optional[float]:
        '''Get the timestamp. Returns None if the message has no header yet.'''
        if hasattr(self, '_header'):
            return self._header.timestamp
        return None
    
class immutable_attr():
    '''Describes an immutable attribute. The type should be already known at run time, that is,
    included in the class attribute definition of the message (and therefore, it does not need
    to be type checked).
    Additionally, to prevent accidental modification of python mutable types, which may lead to 
    an inconsistent state, it checks whether the attribute is of a immutable type before 
    returning the (reference of) the desired object and if the object is mutable, it returns a 
    deep copy.'''

    def __init__(self, doc : str) -> None:
        self.__doc__ = doc

    def __set_name__(self, owner : Any, name : str):
        self._name = '_' + name

    def __get__(self, instance : Any, owner : Any):
        if instance is None: #some hacky thing to allow docstrings
            return self

        if isinstance(getattr(instance, self._name), (int, float, str, bool, tuple)):
            return getattr(instance, self._name)
        else:
            return copy.deepcopy(getattr(instance, self._name))

    def __set__(self, owner : Any, value : Any):
        raise AttributeError('Attribute \'{}\' of {} cannot be modified'.format(self._name, type(owner)))
    
    def __delete__(self, __name: str) -> None:
        raise NotImplementedError

class mutable_attr():
    '''Describes a mutable attribute. The type should be already known at run time, that is,
    included in the class attribute definition of the message.
    Additionally, to prevent accidental modification of python mutable types, which may lead to 
    an inconsistent state, it checks whether the attribute is of a immutable type before 
    returning the (reference of) the desired object and if the object is mutable, it returns a 
    deep copy (new and different reference).
    Since it can still be re-assigned, it will be type checked when this operation is carried out.

    Realization: Inside a message: all attributes are immutable, fields attributes are immutable
    fields contents are mutable.
    
    Also, there is no pythonic way of *prohibiting* (ex: type checker) the assignment of a wrong 
    type, which means it *will* fail. So, failure at the descriptor or failure at the packing 
    step... Are failures/exceptions during runtime either way => bad. I'll leave this implementation
    anyway, since this might give better error messages, but it is, all in all, pointless.
    '''

    def __init__(self, field_def : dict, doc : str) -> None:
        self._field_def = field_def
        self.__doc__ = doc

    def __set_name__(self, owner : Any, name : str):
        self._priv_name = '_' + name
        self._owner = owner

    def __get__(self, instance : Any, owner : Any) -> Any:
        if instance is None: #some hacky thing to allow docstrings
            return self

        # return bare attribute if it is immutable.
        if isinstance(getattr(instance, self._priv_name), (int, float, str, bool, tuple)):
            return getattr(instance, self._priv_name)
        else:
            return copy.deepcopy(getattr(instance, self._priv_name))

    def __set__(self, obj : Any, value : Any) -> None:
        '''Performs type and boundary checks and throws exceptions'''
        set_value = value
        attribute_type = imc_types.get(self._field_def.get('type', None), None)

        if not attribute_type:
            raise KeyError('Could not find a type declaration for {} in given IMC definition'.format(self._priv_name[1:]))

        # Special and only case of upcasting internally allowed.
        if isinstance(set_value, int) and attribute_type == float:
            set_value = float(set_value)

        if isinstance(set_value, attribute_type):
            '''Obs: Should type casting be implemented? eg.: float -> int
            Type casting/coercion will not be implemented to avoid reinforcing bad
            practices and increase transparency.
            
            On a 2nd thought, we may allow SOME upcasting, in particular, int -> float'''
            
            # if it is a list, check its elements types.
            if isinstance(set_value, imc_types['message-list']):
                
                for t in set_value: # type: ignore
                    # Check data type
                    if not isinstance(t, imc_types['message']):
                        raise ValueError('Cannot assign {} to attribute \'{}\'. Expected: {} of {}'.format(
                        type(t), self._priv_name[1:], imc_types[self._field_def['type']], 
                        imc_types['message']))
                    
                    # (I have to check the message group?)
                    # Check message-type 
                    if  self._field_def.get('message-type', None) is not None \
                                    and t.Attributes.abbrev != self._field_def.get('message-type', None):
                        raise ValueError('Cannot have {} in the list of attribute \'{}\'. Expected: {} of messages of type \'{}\''.format(
                        type(t), self._priv_name[1:], imc_types[self._field_def['type']],
                        self._field_def.get('message-type', None)))

            # if it is field, check its validity, according to the IMC XML:
            if 'min' in self._field_def:
                if set_value < self._field_def['min']:
                    raise ValueError('The minimum value for attribute {} is {}. Cannot assign {}.'.format(
                        self._priv_name[1:], self._field_def['min'], set_value
                    ))

            if 'max' in self._field_def:
                if set_value > self._field_def['max']:
                    raise ValueError('The maximum value for attribute \'{}\' is {}. Cannot assign {}.'.format(
                        self._priv_name[1:], self._field_def['max'], set_value
                    ))
            
            # Check if its enumerated or bitfield
            if self._field_def.get('unit', None) == 'Enumerated':
                # Tries to get definition from the owner class. If 'enum-def' exists, it refers to a 
                # global definition; returns None, otherwise
                enum_def = self._field_def.get('enum-def', None) 
                
                # if it is global, get class from file. Else, get definition from owner class
                enum_def = getattr(imc_enums, enum_def) if enum_def else getattr(self._owner, self._priv_name[1:].upper())

                set_value = enum_def(set_value)

            if self._field_def.get('unit', None) == 'Bitfield':
                bitdef = self._field_def.get('bitfield-def', None)
                
                bitdef = getattr(imc_bitf, bitdef) if bitdef else getattr(self._owner, self._priv_name[1:].upper())

                set_value = bitdef(set_value)

            # check the size (or crop the object at serialization?)
            setattr(obj, self._priv_name, set_value)
        else:
            raise AttributeError('Cannot assign {} to {}. Expected: {}'.format(
                type(set_value), self._priv_name[1:], imc_types[self._field_def['type']]))
        

import numpy as np 

# Usefull for applying offset from meters to degrees
c_wgs84_a = 6378137.0
c_wgs84_b = 6356752.3142
c_wgs84_e2 = 0.00669437999013
c_wgs84_ep2 = 0.00673949674228
c_wgs84_f = 0.0033528106647475

class locationType():

    def __init__(self):

        self.lat = 0
        self.lon = 0
        self.depth = 0
        self.alt = 0

        self.roll = 0 
        self.pitch = 0
        self.yaw = 0
    
        self.north_offset = 0 
        self.east_offset = 0 
        self.down_offset = 0 

        self.time = 0

    def fill_it(self, msg):
        
        self.time = msg._header.timestamp
        self.lat = msg.lat 
        self.lon = msg.lon

        self.depth = msg.depth
        self.alt = msg.alt

        self.roll = msg.phi
        self.pitch = msg.theta 
        self.yaw = msg.psi

        self.u = msg.u 
        self.v = msg.v 
        self.w = msg.w

        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

    def set_position(self, pos : 'locationType'):

        self.lat = pos.lat
        self.lon = pos.lon
        self.depth = pos.depth

    
    # Add offsets to (x,y,z)
    def get_offsets(self, x, y, z):

        self.north_offset = self.north_offset + x
        self.east_offset = self.east_offset + y
        self.down_offset = self.down_offset + z

    def translate_positions(self):


        # Translate WGSM coordinates to ECEF so we can add x,y displacement
        x_ecef, y_ecef, z_ecef = toECEF(self.lat,self.lon,self.depth)

        p = np.sqrt(x_ecef**2 + y_ecef**2)

        phi = np.arctan2(z_ecef, p)

        slon = np.sin(self.lon)
        clon = np.cos(self.lon)
        sphi = np.sin(phi)
        cphi = np.cos(phi)
        
        # Add the displacement
        x_ecef = x_ecef + (-slon*self.east_offset - clon*sphi*self.north_offset - clon*cphi*self.down_offset)
        y_ecef = y_ecef + (clon*self.east_offset - slon*sphi*self.north_offset -slon*cphi*self.down_offset)
        z_ecef = z_ecef + (cphi*self.north_offset - sphi*self.down_offset)
        
        lld = []
        lld = [x_ecef, y_ecef, z_ecef]

        return lld

    # Using the coordinates and the offset we check the actual distance between two locations        
    def getHorizontalDistanceInMeters(self, otherLocation : 'locationType'):
        
        displacements = []
        displacements.extend(WGS84displacement(otherLocation.lat, otherLocation.lon, otherLocation.depth, self.lat, self.lon, self.depth))
        
        return np.sqrt(displacements[0]**2 + displacements[1]**2) 
    
    # Get the (x,y,z) displacements
    def getWGS84displacement(self, otherLocation : 'locationType'):

        displacements = []
        displacements.extend(WGS84displacement(otherLocation.lat, otherLocation.lon, otherLocation.depth, self.lat, self.lon, self.depth))
        
        return displacements
    

def computeRN(lat):

    lat_sin = np.sin(lat)
    return (c_wgs84_a / np.sqrt(1 - c_wgs84_e2 * (lat_sin**2)))

def toECEF(lat, lon, depth):

    cos_lat = np.cos(lat)
    sin_lat = np.sin(lat)
    cos_lon = np.cos(lon)
    sin_lon = np.sin(lon)
    
    # Compute the radious of earth at this given latitude
    rn = computeRN(lat)

    x = (rn - depth) * cos_lat * cos_lon
    y = (rn - depth) * cos_lat * sin_lon
    z = ( ( (1.0 - c_wgs84_e2) * rn) - depth) * sin_lat
    return x,y,z   

def n_rad(lat):

    lat_sin = np.sin(lat)
    return c_wgs84_a / np.sqrt(1 - c_wgs84_e2*(lat_sin**2))

# comment this once given the chance
def fromECEF(x, y, z):

    p = np.sqrt(x**2 + y**2) 
    lon = np.arctan2(y, x)
    lat = np.arctan2(z / p, 0.01)

    n = n_rad(lat)
    depth = p / np.cos(lat) - n

    old_depth = -1e-9
    num = z / p

    while(np.abs(depth - old_depth) > 1e-4):

        old_depth = depth

        den =  1 - c_wgs84_e2 * n / (n + depth)
        lat = np.arctan2(num, den)
        n = n_rad(lat)
        depth = p / np.cos(lat) - n

    return lat, lon, depth 

def WGS84displacement(latDegrees1, lonDegrees1, depth1, latDegrees2, lonDegrees2, depth2):
    
    cs1 = []
    cs2 = []

    # Get the coordinates to ECEF format
    cs1.extend(toECEF(latDegrees1, lonDegrees1, depth1))
    cs2.extend(toECEF(latDegrees2, lonDegrees2, depth2))

    # Calculate the displacement between the two points 
    ox = cs2[0] - cs1[0]
    oy = cs2[1] - cs1[1]
    oz = cs2[2] - cs1[2]

    slat = np.sin(latDegrees1)
    clat = np.cos(latDegrees1)
    slon = np.sin(lonDegrees1)
    clon = np.cos(lonDegrees1)

    ret = []

    ret.append(-slat * clon * ox - slat * slon * oy + clat * oz)
    ret.append(-slon * ox + clon * oy)
    ret.append(depth1 - depth2)

    return ret 


