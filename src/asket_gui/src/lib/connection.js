// The WebSocket client and the store behind it.
//
// Three responsibilities, and one of them is a safety rule.
//
// 1. Reconnect. The link will drop. Reconnection is automatic with backoff, and
//    while it is down the UI keeps showing the last values it had — clearly
//    marked as old. Blanking the screen would hide the last known position at
//    exactly the moment it becomes most valuable.
//
// 2. Estimate clock skew, so data age is honest (safety rule 7). Every frame
//    carries `source_utc_ms` (when the value was produced) and `server_utc_ms`
//    (when it was sent). Age is computed against the *server's* clock, not the
//    laptop's, because a laptop with a wrong clock would otherwise render every
//    value as fresh or as hours old, and an operator would have no way to tell.
//
//    Skew is estimated as max(server_utc_ms - local receive time) over a recent
//    window. Transmission delay only ever makes a frame look older, so the
//    maximum over several samples is the sample that suffered the least delay,
//    and therefore the best estimate of true skew.
//
// 3. Never fabricate. A stream with no data has no data; it does not have zero.

const RECONNECT_BACKOFF_MS = [500, 1000, 2000, 4000, 8000, 15000];
const SKEW_WINDOW = 32;

function initialState() {
  return {
    connected: false,
    connecting: true,
    attempts: 0,
    lastError: null,
    hello: null,
    profile: { profile: 'full', manual: false, reason: 'connecting' },
    subscriptions: {},
    estimatedBytesPerS: 0,
    streams: {},
    alarms: [],
    commands: {},
    skewMs: 0,
    lastMessageAt: null,
  };
}

export class Connection {
  constructor(url) {
    this.url =
      url ||
      `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.host}/ws`;
    this.state = initialState();
    this.listeners = new Set();
    this.desired = [];
    this.skewSamples = [];
    this.ws = null;
    this.reconnectTimer = null;
    this.closed = false;
    this.tickTimer = setInterval(() => this.#emit(), 250);
  }

  // -- React glue -------------------------------------------------------

  subscribeToStore = (listener) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };

  getSnapshot = () => this.state;

  #set(partial) {
    this.state = { ...this.state, ...partial };
    this.#emit();
  }

  #emit() {
    for (const listener of this.listeners) listener();
  }

  // -- lifecycle --------------------------------------------------------

  connect() {
    this.closed = false;
    this.#open();
  }

  close() {
    this.closed = true;
    clearInterval(this.tickTimer);
    clearTimeout(this.reconnectTimer);
    if (this.ws) this.ws.close();
  }

  #open() {
    this.#set({ connecting: true });
    let ws;
    try {
      ws = new WebSocket(this.url);
    } catch (error) {
      this.#scheduleReconnect(error.message);
      return;
    }
    this.ws = ws;

    ws.onopen = () => {
      this.#set({ connected: true, connecting: false, attempts: 0, lastError: null });
      if (this.desired.length) this.subscribe(this.desired);
    };

    ws.onmessage = (event) => {
      let message;
      try {
        message = JSON.parse(event.data);
      } catch {
        return; // a malformed frame is not worth tearing the session down for
      }
      this.#handle(message);
    };

    ws.onerror = () => {
      // onclose always follows; recording the reason there avoids reporting
      // the same drop twice.
    };

    ws.onclose = () => {
      this.#set({ connected: false });
      this.#scheduleReconnect('connection closed');
    };
  }

  #scheduleReconnect(reason) {
    if (this.closed) return;
    const attempts = this.state.attempts + 1;
    const delay = RECONNECT_BACKOFF_MS[Math.min(attempts - 1, RECONNECT_BACKOFF_MS.length - 1)];
    this.#set({ connected: false, connecting: true, attempts, lastError: reason });
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = setTimeout(() => this.#open(), delay);
  }

  // -- inbound ----------------------------------------------------------

  #handle(message) {
    const now = Date.now();
    this.state.lastMessageAt = now;

    if (typeof message.server_utc_ms === 'number') {
      this.skewSamples.push(message.server_utc_ms - now);
      if (this.skewSamples.length > SKEW_WINDOW) this.skewSamples.shift();
      this.state.skewMs = Math.max(...this.skewSamples);
    }

    switch (message.type) {
      case 'hello':
        this.#set({ hello: message, profile: message.profile });
        break;

      case 'subscribed': {
        const subscriptions = { ...this.state.subscriptions };
        for (const entry of message.streams || []) subscriptions[entry.name] = entry;
        this.#set({
          subscriptions,
          profile: message.profile || this.state.profile,
          estimatedBytesPerS: message.estimated_bytes_per_s ?? 0,
        });
        break;
      }

      case 'profile':
        this.#set({
          profile: {
            profile: message.profile,
            manual: message.manual,
            reason: message.reason,
          },
        });
        break;

      case 'data': {
        const streams = { ...this.state.streams };
        streams[message.stream] = {
          payload: message.payload,
          sourceUtcMs: message.source_utc_ms,
          serverUtcMs: message.server_utc_ms,
          detail: message.detail,
          receivedAt: now,
        };
        this.#set({ streams });
        break;
      }

      case 'alarms':
        this.#set({ alarms: message.active || [] });
        break;

      case 'command_result': {
        const commands = { ...this.state.commands };
        commands[message.id] = message;
        this.#set({ commands });
        break;
      }

      case 'ping':
        // Answer immediately so the server's round-trip measurement — which is
        // what automatic profile selection runs on — reflects the link and not
        // this tab's render loop.
        this.send({ type: 'pong', id: message.id });
        break;

      default:
        break;
    }
  }

  // -- outbound ---------------------------------------------------------

  send(message) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
      return true;
    }
    return false;
  }

  subscribe(streams) {
    this.desired = streams;
    return this.send({ type: 'subscribe', streams });
  }

  unsubscribe(names) {
    this.desired = this.desired.filter((s) => !names.includes(s.name));
    return this.send({ type: 'unsubscribe', streams: names });
  }

  setProfile(profile) {
    return this.send({ type: 'set_profile', profile });
  }

  command(name, args = {}) {
    const id = `${name}-${Date.now().toString(36)}`;
    const commands = { ...this.state.commands };
    // Recorded locally as pending so the button can show it immediately. This
    // is NOT the vessel's state — nothing displayed as vessel state changes
    // until the server reports a confirmation (safety rule 4).
    commands[id] = { id, name, args, status: 'pending', detail: 'sending…' };
    this.#set({ commands });
    if (!this.send({ type: 'command', id, name, args })) {
      commands[id] = { ...commands[id], status: 'failed', detail: 'no link to the vessel' };
      this.#set({ commands: { ...commands } });
    }
    return id;
  }

  // -- derived ----------------------------------------------------------

  // The server's current time, as best we can estimate it.
  serverNow() {
    return Date.now() + this.state.skewMs;
  }
}

// Age of a stream's value, in milliseconds, or null if it has never arrived.
export function streamAgeMs(state, name) {
  const entry = state.streams[name];
  if (!entry) return null;
  return Date.now() + state.skewMs - entry.sourceUtcMs;
}

export function streamPayload(state, name) {
  return state.streams[name]?.payload ?? null;
}
