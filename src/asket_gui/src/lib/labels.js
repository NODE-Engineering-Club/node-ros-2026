// Wording that is a safety decision, not a style choice.
//
// Rule 3 of docs/safety.md: the software ESTOP is a soft latch only. It is
// labelled "Cut propulsion" and never "Emergency stop", because the hardware
// killswitch and RC channel 8 are the emergency stop, and no operator should
// ever come to rely on a piece of software as one.
//
// Every user-visible string for a command lives here so that the wording is one
// decision in one place, and so that a check can assert what is absent as well
// as what is present.

export const MODE_LABELS = {
  MANUAL: 'Manual',
  AUTONOMOUS: 'Autonomous',
  ESTOP: 'Propulsion cut',
  UNKNOWN: 'Unknown',
};

export const COMMAND_LABELS = {
  set_mode_MANUAL: 'Manual',
  set_mode_AUTONOMOUS: 'Autonomous',
  cut_propulsion: 'Cut propulsion',
};

// The second step of the two-step confirmation. Says what will happen, in the
// vessel's terms, not "Are you sure?".
export const CONFIRM_PROMPTS = {
  set_mode_MANUAL: 'Hand control to the RC transmitter?',
  set_mode_AUTONOMOUS: 'Let the vessel drive itself?',
  cut_propulsion:
    'Cut propulsion? This is a software latch. The hardware killswitch and RC channel 8 are separate and always work.',
};

export const COMMAND_STATUS_LABELS = {
  pending: 'Waiting for the vessel…',
  confirmed: 'Confirmed by the vessel',
  failed: 'Not applied',
};

export const LINK_LABELS = {
  ethernet: 'Ethernet',
  wifi: 'WiFi',
  '4g': '4G',
  ltem: 'LTE-M',
  none: 'No link',
};

export const PROFILE_LABELS = {
  full: 'Full',
  reduced: 'Reduced',
  minimal: 'Beacon',
};

export const HEADING_SOURCE_LABELS = {
  gnss_compass: 'GNSS compass',
  magnetometer: 'Magnetometer',
  cog: 'Course over ground',
  none: 'None',
};
