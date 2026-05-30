// Minimal TypeScript definitions for the ROS 2 messages this panel reads and
// publishes. These mirror the on-the-wire field names exactly (e.g. the ROS 2
// builtin_interfaces/Time uses `nanosec`, not `nsec`) so messages serialize
// correctly through the Foxglove WebSocket bridge.
//
// Kept in a dedicated module so additional message types can be added here as
// the panel grows, without cluttering the component code.

/** builtin_interfaces/Time */
export interface Time {
  sec: number;
  nanosec: number;
}

/** std_msgs/Header */
export interface Header {
  stamp: Time;
  frame_id: string;
}

/** sensor_msgs/NavSatStatus (subset) */
export interface NavSatStatus {
  status: number;
  service: number;
}

/** sensor_msgs/NavSatFix (subset of fields used here) */
export interface NavSatFix {
  latitude: number;
  longitude: number;
  altitude?: number;
  status?: NavSatStatus;
}

/** geometry_msgs/Point */
export interface Point {
  x: number;
  y: number;
  z: number;
}

/** geometry_msgs/Quaternion */
export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

/** geometry_msgs/Pose */
export interface Pose {
  position: Point;
  orientation: Quaternion;
}

/** geometry_msgs/PoseArray */
export interface PoseArray {
  header: Header;
  poses: Pose[];
}

/** Foxglove schema name used when advertising the waypoints topic. */
export const POSE_ARRAY_SCHEMA = "geometry_msgs/msg/PoseArray";

/**
 * Frame id used on the published PoseArray to mark the poses as geographic
 * (WGS84) coordinates rather than a Cartesian map frame. By convention here:
 *   position.x = longitude (deg)
 *   position.y = latitude  (deg)
 *   position.z = 0
 * The boat-side consumer is responsible for converting these to the map frame
 * (e.g. via robot_localization /fromLL) before handing them to Nav2.
 */
export const GEO_FRAME_ID = "wgs84";

/** A single user-entered waypoint in decimal degrees. */
export interface Waypoint {
  /** Stable id for React keys and deletion (not the display number). */
  id: number;
  lat: number;
  lon: number;
  /** Optional human-readable label. */
  label?: string;
}

/** Build a current-time builtin_interfaces/Time from the wall clock. */
function nowTime(): Time {
  const ms = Date.now();
  return { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1_000_000 };
}

/**
 * Convert an ordered list of GPS waypoints into a geometry_msgs/PoseArray with
 * longitude in x and latitude in y (see GEO_FRAME_ID). Orientation is identity.
 */
export function waypointsToPoseArray(waypoints: Waypoint[]): PoseArray {
  return {
    header: { stamp: nowTime(), frame_id: GEO_FRAME_ID },
    poses: waypoints.map((w) => ({
      position: { x: w.lon, y: w.lat, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    })),
  };
}
