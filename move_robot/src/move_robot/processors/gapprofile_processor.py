#!/usr/bin/env python3

import rospy
import struct
from io import BytesIO
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from typing import Optional, Dict, Any
from gap_detection.msg import GapProfile

class GapData:
    """
    Structure representing the gap detection message data
    """
    
    def __init__(self):
        # Header information
        self.header = Header()
        
        # Gap width information
        self.current_width = 0.0
        self.nominal_width = 0.003  # 3mm padrão
        self.point_step = 0
        
        # Depth information
        self.depth = 0.0
        self.distance = 0.0
        
        # Edge positions
        self.left_edge = Point()
        self.right_edge = Point()
        
        # Optional fields
        self.is_discontinuous = False
        self.discontinuity_length = 0.0
        self.confidence = 0.0
        self.profile_uniformity = 0.0
        self.edge_sharpness = 0.0
    
    def __str__(self) -> str:
        """String representation of the gap data"""
        return (f"Gap Data:\n"
                f"  Header: seq={self.header.seq}, stamp={self.header.stamp}, frame_id={self.header.frame_id}\n"
                f"  Current Width: {self.current_width:.4f} m\n"
                f"  Nominal Width: {self.nominal_width:.4f} m\n"
                f"  Point Step: {self.point_step}\n"
                f"  Depth: {self.depth:.4f} m\n"
                f"  Distance: {self.distance:.4f} m\n"
                f"  Left Edge: ({self.left_edge.x:.4f}, {self.left_edge.y:.4f}, {self.left_edge.z:.4f})\n"
                f"  Right Edge: ({self.right_edge.x:.4f}, {self.right_edge.y:.4f}, {self.right_edge.z:.4f})")


class GapDetectionMsgUnpacker:
    """
    Class to unpack gap detection messages in ROS 1 Noetic.
    
    This class handles the unpacking of custom gap detection messages
    that contain information about detected gaps including width,
    depth, edge positions, and other relevant metrics.
    """
    
    # Expected message format (based on the commented structure)
    # Format string for struct module:
    # Header: seq (I), stamp.secs (I), stamp.nsecs (I), frame_id (string)
    # current_width (f), nominal_width (f), point_step (B)
    # depth (f), distance (f)
    # left_edge: x (f), y (f), z (f)
    # right_edge: x (f), y (f), z (f)
    # Optional fields (if implemented):
    # is_discontinuous (?), discontinuity_length (f), confidence (f),
    # profile_uniformity (f), edge_sharpness (f)
    
    def __init__(self, verbose: bool = False):
        """
        Constructor for GapDetectionMsgUnpacker
        
        Args:
            verbose: If True, print debug information
        """
        self.verbose = verbose
        self.latest_gap_data = GapData()
        self.new_data_available = False
        
        # Expected message size (approximate, without optional fields)
        # Header: 3 uint32 + string (variable) + 3 floats + 1 uint8 + 2 floats + 6 floats for edges
        self.expected_fields = [
            'header.seq', 'header.stamp.secs', 'header.stamp.nsecs', 'header.frame_id',
            'current_width', 'nominal_width', 'point_step',
            'depth', 'distance',
            'left_edge.x', 'left_edge.y', 'left_edge.z',
            'right_edge.x', 'right_edge.y', 'right_edge.z'
        ]
        
        if self.verbose:
            rospy.loginfo("GapDetectionMsgUnpacker initialized")
    
    def _gap_callback(self, msg: GapProfile):
        """
        Callback function for the gap detection subscriber
        
        Args:
            msg: The raw message to be unpacked
        """
        if self._parse_message(msg):
            self.new_data_available = True
            
            if self.verbose:
                self.print_gap_data()

    def _get_struct_fmt(self, fields, field_names=None):
        fmt = '<'

        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt
    
    def _parse_message(self, msg: GapProfile) -> bool:
        """
        Parse the message data from serialized form
        
        Args:
            msg: The raw message to parse
            
        Returns:
            True if parsing was successful
        """
        try:
            # Access the serialized data
            #fmt = self._get_struct_fmt(msg.fields, self.expected_fields)
    
            serialized_data = msg.serialize(buf) # This gives us the raw bytes
            
            if self.verbose:
                rospy.loginfo(f"Received message with {len(serialized_data)} bytes")
            
            # Create a new GapData instance
            gap_data = GapData()
            
            # Parse the message based on expected format
            # This is a simplified parsing approach - you may need to adjust
            # based on your actual message serialization
            
            # For demonstration, we'll use a manual parsing approach
            # In practice, you might want to use the actual message definition
            
            # Example parsing (adjust based on your actual message format):
            offset = 0
            
            # Parse Header
            # seq (uint32)
            if len(serialized_data) >= offset + 4:
                gap_data.header.seq = struct.unpack('<I', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # stamp.secs (uint32)
            if len(serialized_data) >= offset + 4:
                stamp_secs = struct.unpack('<I', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # stamp.nsecs (uint32)
            if len(serialized_data) >= offset + 4:
                stamp_nsecs = struct.unpack('<I', serialized_data[offset:offset+4])[0]
                offset += 4
                
                # Set the stamp
                gap_data.header.stamp.secs = stamp_secs
                gap_data.header.stamp.nsecs = stamp_nsecs
            
            # frame_id (string - length prefixed)
            if len(serialized_data) >= offset + 4:
                frame_id_len = struct.unpack('<I', serialized_data[offset:offset+4])[0]
                offset += 4
                
                if len(serialized_data) >= offset + frame_id_len:
                    gap_data.header.frame_id = serialized_data[offset:offset+frame_id_len].decode('utf-8')
                    offset += frame_id_len
            
            # current_width (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.current_width = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # nominal_width (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.nominal_width = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # point_step (uint8)
            if len(serialized_data) >= offset + 1:
                gap_data.point_step = struct.unpack('<B', serialized_data[offset:offset+1])[0]
                offset += 1
            
            # depth (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.depth = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # distance (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.distance = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # left_edge.x (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.left_edge.x = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # left_edge.y (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.left_edge.y = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # left_edge.z (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.left_edge.z = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # right_edge.x (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.right_edge.x = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # right_edge.y (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.right_edge.y = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # right_edge.z (float32)
            if len(serialized_data) >= offset + 4:
                gap_data.right_edge.z = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # Optional: Parse additional fields if they exist in the message
            # Check if there's more data for optional fields
            remaining_bytes = len(serialized_data) - offset
            
            if remaining_bytes >= 1:  # is_discontinuous (bool)
                gap_data.is_discontinuous = struct.unpack('<?', serialized_data[offset:offset+1])[0]
                offset += 1
            
            if remaining_bytes >= 5:  # discontinuity_length (float32)
                gap_data.discontinuity_length = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            if remaining_bytes >= 5:  # confidence (float32)
                gap_data.confidence = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            if remaining_bytes >= 5:  # profile_uniformity (float32)
                gap_data.profile_uniformity = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            if remaining_bytes >= 5:  # edge_sharpness (float32)
                gap_data.edge_sharpness = struct.unpack('<f', serialized_data[offset:offset+4])[0]
                offset += 4
            
            # Update the latest data
            self.latest_gap_data = gap_data
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to parse gap detection message: {e}")
            return False
    
    def get_latest_gap_data(self) -> GapData:
        """
        Get the latest gap data
        
        Returns:
            GapData object with the latest gap information
        """
        return self.latest_gap_data
    
    def is_new_data_available(self) -> bool:
        """
        Check if new data is available
        
        Returns:
            True if new data has been received since last check
        """
        return self.new_data_available
    
    def reset_new_data_flag(self):
        """Reset the new data flag"""
        self.new_data_available = False
    
    def print_gap_data(self):
        """Print gap data to console (for debugging)"""
        rospy.loginfo("\n" + str(self.latest_gap_data))
    
    def shutdown(self):
        """Clean shutdown of the subscriber"""
        if self.verbose:
                rospy.loginfo("Gap detection subscriber shut down")


# # Example usage
# if __name__ == '__main__':
#     try:
#         # Initialize ROS node
#         rospy.init_node('gap_detection_unpacker_example', anonymous=True)
        
#         # Create unpacker instance
#         unpacker = GapDetectionMsgUnpacker(verbose=True)
        
#         # Example rate for processing
#         rate = rospy.Rate(10)  # 10 Hz
        
#         while not rospy.is_shutdown():
#             if unpacker.is_new_data_available():
#                 # Get the latest gap data
#                 gap_data = unpacker.get_latest_gap_data()
                
#                 # Process the data (example)
#                 if gap_data.current_width > 0:
#                     rospy.loginfo(f"Gap detected with width: {gap_data.current_width:.3f} m")
#                     rospy.loginfo(f"Depth: {gap_data.depth:.3f} m")
                
#                 # Reset the flag after processing
#                 unpacker.reset_new_data_flag()
            
#             rate.sleep()
            
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         if 'unpacker' in locals():
#             unpacker.shutdown()