from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_object_pb2 import MovingObject


def build_static_target(width, length, height, x, y, roll, pitch, yaw,
                        timestamp_s, host_vehicle_id=0):
    """Build a GroundTruth message containing a single stationary object.

    Replaces the deprecated SensorDataSeries-based approach.

    Args:
        timestamp_s: Timestamp in seconds (e.g. from bag.get_start_time()).
            Mandatory per OSI spec (GroundTruth.timestamp is_set rule).
        host_vehicle_id: Identifier for the host vehicle. A matching
            moving_object entry is created so the reference is valid
            (required by Lichtblick FrameTransforms).

    Returns:
        A populated GroundTruth protobuf message.
    """
    gt = GroundTruth()
    gt.timestamp.seconds = int(timestamp_s)
    gt.timestamp.nanos = int((timestamp_s % 1) * 1e9)
    gt.host_vehicle_id.value = host_vehicle_id

    # host_vehicle_id must reference a moving_object entry (OSI spec).
    # The ego vehicle defines the coordinate origin — all sensor mounting
    # positions and target RTK offsets are relative to it.
    ego = gt.moving_object.add()
    ego.id.value = host_vehicle_id
    ego.type = MovingObject.TYPE_VEHICLE
    ego.base.position.x = 0.0
    ego.base.position.y = 0.0
    ego.base.position.z = 0.0
    ego.base.orientation.yaw = 0.0
    ego.base.orientation.pitch = 0.0
    ego.base.orientation.roll = 0.0
    # Dimensions must be present (Lichtblick reads base.dimension.width).
    # Set to zero when actual vehicle dimensions are unknown.
    ego.base.dimension.width = 0.0
    ego.base.dimension.length = 0.0
    ego.base.dimension.height = 0.0
    # bbcenter_to_rear must be present for Lichtblick to build the
    # ego_vehicle_rear_axle frame. Set to zero when unknown
    # (rear axle = bb center).
    ego.vehicle_attributes.bbcenter_to_rear.x = 0.0
    ego.vehicle_attributes.bbcenter_to_rear.y = 0.0
    ego.vehicle_attributes.bbcenter_to_rear.z = 0.0

    obj = gt.stationary_object.add()
    obj.id.value = host_vehicle_id + 1
    obj.base.dimension.width = width
    obj.base.dimension.length = length
    obj.base.dimension.height = height
    obj.base.position.x = x
    obj.base.position.y = y
    # BaseStationary.position is the bounding-box center (OSI spec).
    # For ground-resting objects, z = height / 2.
    obj.base.position.z = height / 2.0
    obj.base.orientation.roll = roll
    obj.base.orientation.pitch = pitch
    obj.base.orientation.yaw = yaw
    # classification must be present — the Lichtblick converter accesses
    # classification.color without a null guard (sceneUpdateConverter.ts:120).
    # 0 = UNKNOWN for all sub-fields, which renders as gray.
    obj.classification.color = 0
    obj.classification.type = 0
    obj.classification.material = 0
    obj.classification.density = 0
    return gt
