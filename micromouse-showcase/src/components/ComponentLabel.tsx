import { componentById } from '../config/components';
import type { ComponentId } from '../types/showcase';

interface ComponentLabelProps {
  selected: ComponentId | null;
  onClose: () => void;
}

export function ComponentLabel({ selected, onClose }: ComponentLabelProps) {
  if (!selected) return null;
  const component = componentById[selected];

  return (
    <aside className="component-card" aria-live="polite" data-testid="component-detail">
      <div className="component-card__rule" style={{ backgroundColor: component.accent }} />
      <div>
        <span className="component-card__code">SYS / {component.shortLabel}</span>
        <h2>{component.title}</h2>
        <p>{component.description}</p>
      </div>
      <button className="icon-button" type="button" onClick={onClose} aria-label="Close component details">
        ×
      </button>
    </aside>
  );
}
