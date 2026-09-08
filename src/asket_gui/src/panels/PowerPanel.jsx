import { Chip, Panel, Row, Rows } from '../components/Panel.jsx';
import { PanelAge, Value } from '../components/Value.jsx';
import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { duration, num, pct } from '../lib/format.js';

/**
 * Battery, and the comparison that actually matters.
 *
 * With one sonar unit covering one side only, the survey distance roughly
 * doubles. So the useful question is not "how much charge is left" but "is
 * there enough to finish", and the panel answers that directly rather than
 * leaving an operator to do the arithmetic on a beach.
 */
export function PowerPanel({ state }) {
  const power = streamPayload(state, 'power');
  const ageMs = streamAgeMs(state, 'power');
  const rateHz = state.subscriptions.power?.rate_hz;

  const soc = power?.state_of_charge;
  const level = soc === undefined ? '' : soc < 0.12 ? 'alarm' : soc < 0.25 ? 'warn' : 'ok';
  const canFinish = power?.can_finish_survey;
  const margin = power?.endurance_margin_s;

  return (
    <Panel title="Power" aside={<PanelAge ageMs={ageMs} rateHz={rateHz} />}>
      <div style={{ display: 'flex', alignItems: 'baseline', gap: 10 }}>
        <span className="big">
          <Value ageMs={ageMs} rateHz={rateHz} showAge={false}>
            {pct(soc)}
          </Value>
        </span>
        <Chip level={level}>{num(power?.voltage, 1, ' V')}</Chip>
      </div>

      <Rows>
        <Row label="Current">{num(power?.current, 1, ' A')}</Row>
        <Row label="Draw">{num(power?.power_w, 0, ' W')}</Row>
        <Row label="Energy left">{num(power?.remaining_wh, 0, ' Wh')}</Row>
        <Row label="Endurance">{duration(power?.endurance_s)}</Row>
      </Rows>

      {power?.survey_remaining_m !== undefined ? (
        <>
          <Rows>
            <Row label="Survey left">{num(power.survey_remaining_m / 1000, 2, ' km')}</Row>
            <Row label="Time to finish">{duration(power.survey_remaining_s)}</Row>
            <Row label="Margin">
              <span className={canFinish ? '' : 'errline'}>
                {margin === undefined ? '—' : `${margin >= 0 ? '+' : '−'}${duration(Math.abs(margin))}`}
              </span>
            </Row>
          </Rows>
          <p
            className={canFinish ? 'hint' : 'errline'}
            style={{ marginBottom: 0, marginTop: 6 }}
          >
            {canFinish
              ? 'Enough charge to finish the planned survey.'
              : 'Not enough charge to finish the planned survey. Shorten the pattern or plan a battery swap.'}
          </p>
        </>
      ) : (
        <p className="hint" style={{ marginBottom: 0, marginTop: 6 }}>
          Remaining survey distance needs the vessel to be moving before it can be
          compared against endurance.
        </p>
      )}
    </Panel>
  );
}
