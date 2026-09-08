import { usePanelOpen } from '../lib/collapse.js';

/**
 * A cockpit panel, optionally with a collapsible detail section.
 *
 * `children` are the detail — the values that explain rather than trigger
 * action, folded away by default. `summary` is what stays on screen when the
 * detail is folded, and it must be a **verdict**, not a number: "Enough charge
 * to finish the planned survey" tells an operator what to do, "95%" does not.
 *
 * `forceOpen` unfolds the panel regardless of the operator's preference, and
 * `forceReason` says why on the header. That is the rule that makes collapsing
 * safe at all — without it, folding detail away is just a way to hide faults.
 * The preference is not overwritten while it is forced: when the condition
 * clears, the panel returns to however the operator had left it.
 */
export function Panel({
  title,
  aside,
  children,
  id,
  summary,
  collapsible = false,
  defaultOpen = false,
  forceOpen = false,
  forceReason = '',
}) {
  const [openPref, toggle] = usePanelOpen(id, defaultOpen);
  const open = forceOpen || openPref;

  if (!collapsible) {
    return (
      <section className="panel">
        <h2>
          <span>{title}</span>
          {aside}
        </h2>
        {summary}
        {children}
      </section>
    );
  }

  return (
    <section className={`panel ${open ? '' : 'collapsed'}`}>
      <h2>
        <button
          type="button"
          className="disclose"
          onClick={toggle}
          disabled={forceOpen}
          aria-expanded={open}
          title={
            forceOpen
              ? `Held open: ${forceReason}. It will fold away again when that clears.`
              : open
                ? `Hide ${title} detail`
                : `Show ${title} detail`
          }
        >
          <span className="caret">{open ? '▾' : '▸'}</span>
          <span>{title}</span>
        </button>
        {aside}
      </h2>
      {summary}
      {forceOpen && forceReason && <p className="forced-note">{forceReason}</p>}
      {open && <div className="panel-detail">{children}</div>}
    </section>
  );
}

export function Rows({ children }) {
  return <dl className="rows">{children}</dl>;
}

export function Row({ label, children }) {
  return (
    <>
      <dt>{label}</dt>
      <dd>{children}</dd>
    </>
  );
}

export function Chip({ level = '', children, title }) {
  return (
    <span className={`chip ${level}`} title={title}>
      <span className="dot" />
      {children}
    </span>
  );
}
