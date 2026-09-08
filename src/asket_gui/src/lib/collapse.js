import { useCallback, useEffect, useState } from 'react';

/**
 * Which panels the operator has opened, remembered across reloads.
 *
 * The reason panels collapse at all: roughly forty-five numbers were on screen
 * at once, and nobody monitors forty-five numbers. An operator watches five and
 * ignores the rest, so the other forty were burying the five that mattered.
 *
 * The rule for what stays visible is not volume, it is: **would a change in
 * this value require somebody to act?** Values that trigger action stay out;
 * values that explain or detail one fold away.
 *
 * Two things make that safe, and both are enforced in `Panel`:
 *
 * 1. A collapsed panel shows a **verdict**, not raw data. "Enough charge to
 *    finish the planned survey" is worth more than "95%", because 95% does not
 *    say whether it is enough.
 * 2. Anything off-nominal **forces itself open**. A fault behind a disclosure
 *    triangle is a hidden fault, and hiding faults is the only way this change
 *    could make the interface worse rather than better.
 */
const KEY = 'asket.panels.open';

function read() {
  try {
    const raw = window.localStorage.getItem(KEY);
    return raw ? JSON.parse(raw) : {};
  } catch {
    // A browser with storage disabled must still render a usable cockpit.
    return {};
  }
}

function write(state) {
  try {
    window.localStorage.setItem(KEY, JSON.stringify(state));
  } catch {
    // Losing the preference is survivable; failing to render is not.
  }
}

export function usePanelOpen(id, defaultOpen = false) {
  const [open, setOpen] = useState(() => {
    const stored = read()[id];
    return stored === undefined ? defaultOpen : stored;
  });

  useEffect(() => {
    if (!id) return;
    const stored = read();
    if (stored[id] === open) return;
    write({ ...stored, [id]: open });
  }, [id, open]);

  return [open, useCallback(() => setOpen((v) => !v), [])];
}
