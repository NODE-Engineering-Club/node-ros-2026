import { useEffect, useRef, useState } from 'react';

/**
 * Two-step confirmation (safety rule 5).
 *
 * Click once and the button asks; click again and it sends. The confirmation
 * step times out on its own, so a button armed by an accidental click does not
 * sit there waiting to be pressed.
 *
 * The button reflects the *command*, never the vessel. A pending command
 * renders outlined and pulsing — visibly a different thing from a confirmed
 * state, which is displayed elsewhere and only ever comes from the vessel.
 */
export function ConfirmButton({
  label,
  prompt,
  onConfirm,
  disabled,
  danger,
  pending,
  armMs = 5000,
}) {
  const [armed, setArmed] = useState(false);
  const timer = useRef(null);

  useEffect(() => () => clearTimeout(timer.current), []);
  useEffect(() => {
    if (pending && armed) setArmed(false);
  }, [pending, armed]);

  function handleClick() {
    if (armed) {
      clearTimeout(timer.current);
      setArmed(false);
      onConfirm();
      return;
    }
    setArmed(true);
    clearTimeout(timer.current);
    timer.current = setTimeout(() => setArmed(false), armMs);
  }

  const className = [danger ? 'danger' : '', pending ? 'pending' : ''].join(' ').trim();

  return (
    <button
      className={className}
      onClick={handleClick}
      disabled={disabled || pending}
      title={armed ? prompt : label}
    >
      {pending ? 'Waiting…' : armed ? `Confirm: ${prompt}` : label}
    </button>
  );
}
