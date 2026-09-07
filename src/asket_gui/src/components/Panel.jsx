export function Panel({ title, aside, children }) {
  return (
    <section className="panel">
      <h2>
        <span>{title}</span>
        {aside}
      </h2>
      {children}
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
