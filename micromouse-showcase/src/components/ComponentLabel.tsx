import { useEffect, useRef, type CSSProperties } from 'react';
import { componentById } from '../config/components';
import type { ComponentId, ComponentScreenAnchorRef } from '../types/showcase';

interface ComponentLabelProps {
  selected: ComponentId | null;
  anchorRef: ComponentScreenAnchorRef;
  onClose: () => void;
}

const clamp = (value: number, minimum: number, maximum: number) => (
  Math.min(Math.max(value, minimum), Math.max(minimum, maximum))
);

export function ComponentLabel({ selected, anchorRef, onClose }: ComponentLabelProps) {
  const cardRef = useRef<HTMLElement>(null);
  const connectorRef = useRef<SVGSVGElement>(null);
  const lineRef = useRef<SVGLineElement>(null);
  const anchorDotRef = useRef<SVGCircleElement>(null);
  const cardDotRef = useRef<SVGCircleElement>(null);

  useEffect(() => {
    if (!selected) return;

    let animationFrame = 0;

    const updateCallout = () => {
      const card = cardRef.current;
      const connector = connectorRef.current;
      const line = lineRef.current;
      const anchorDot = anchorDotRef.current;
      const cardDot = cardDotRef.current;
      const anchor = anchorRef.current;

      if (!card || !connector || !line || !anchorDot || !cardDot || !anchor) {
        if (card) card.style.visibility = 'hidden';
        if (connector) connector.style.opacity = '0';
        animationFrame = window.requestAnimationFrame(updateCallout);
        return;
      }

      const viewportWidth = window.innerWidth;
      const viewportHeight = window.innerHeight;
      const isOnScreen = anchor.visible
        && anchor.x >= 0
        && anchor.x <= viewportWidth
        && anchor.y >= 0
        && anchor.y <= viewportHeight;

      if (!isOnScreen) {
        card.style.visibility = 'hidden';
        connector.style.opacity = '0';
        animationFrame = window.requestAnimationFrame(updateCallout);
        return;
      }

      card.style.visibility = 'hidden';
      const cardBounds = card.getBoundingClientRect();
      const edge = viewportWidth <= 640 ? 16 : 24;
      const gap = viewportWidth <= 640 ? 28 : 52;
      let cardLeft: number;
      let cardTop: number;
      let lineEndX: number;
      let lineEndY: number;

      if (viewportWidth <= 720) {
        cardLeft = clamp(anchor.x - cardBounds.width / 2, edge, viewportWidth - cardBounds.width - edge);
        const fitsBelow = anchor.y + gap + cardBounds.height <= viewportHeight - edge;
        cardTop = fitsBelow ? anchor.y + gap : anchor.y - gap - cardBounds.height;
        cardTop = clamp(cardTop, edge, viewportHeight - cardBounds.height - edge);
        lineEndX = clamp(anchor.x, cardLeft + 18, cardLeft + cardBounds.width - 18);
        lineEndY = cardTop > anchor.y ? cardTop : cardTop + cardBounds.height;
      } else {
        const placeOnRight = anchor.x <= viewportWidth / 2;
        cardLeft = placeOnRight
          ? anchor.x + gap
          : anchor.x - gap - cardBounds.width;
        cardLeft = clamp(cardLeft, edge, viewportWidth - cardBounds.width - edge);
        cardTop = clamp(
          anchor.y - cardBounds.height / 2,
          edge,
          viewportHeight - cardBounds.height - edge,
        );
        lineEndX = placeOnRight ? cardLeft : cardLeft + cardBounds.width;
        lineEndY = clamp(anchor.y, cardTop + 18, cardTop + cardBounds.height - 18);
      }

      card.style.left = `${Math.round(cardLeft)}px`;
      card.style.top = `${Math.round(cardTop)}px`;
      card.style.visibility = 'visible';
      card.dataset.positioned = 'true';
      connector.style.opacity = '1';

      line.setAttribute('x1', String(anchor.x));
      line.setAttribute('y1', String(anchor.y));
      line.setAttribute('x2', String(lineEndX));
      line.setAttribute('y2', String(lineEndY));
      anchorDot.setAttribute('cx', String(anchor.x));
      anchorDot.setAttribute('cy', String(anchor.y));
      cardDot.setAttribute('cx', String(lineEndX));
      cardDot.setAttribute('cy', String(lineEndY));

      animationFrame = window.requestAnimationFrame(updateCallout);
    };

    animationFrame = window.requestAnimationFrame(updateCallout);
    return () => window.cancelAnimationFrame(animationFrame);
  }, [anchorRef, selected]);

  useEffect(() => {
    if (!selected) return;

    const closeOnOutsideClick = (event: MouseEvent) => {
      const target = event.target;
      if (!(target instanceof Element)) return;
      if (cardRef.current?.contains(target)) return;
      if (target.closest('[data-component-trigger]')) return;
      if (target.closest('[data-testid="showcase-canvas"]')) return;
      onClose();
    };

    document.addEventListener('click', closeOnOutsideClick);
    return () => document.removeEventListener('click', closeOnOutsideClick);
  }, [onClose, selected]);

  if (!selected) return null;
  const component = componentById[selected];
  const accentStyle = { '--component-accent': component.accent } as CSSProperties;

  return (
    <div className="component-callout" style={accentStyle}>
      <svg
        ref={connectorRef}
        className="component-callout__connector"
        data-testid="component-connector"
        aria-hidden="true"
      >
        <line ref={lineRef} className="component-callout__line" />
        <circle ref={anchorDotRef} className="component-callout__dot component-callout__dot--anchor" r="5" />
        <circle ref={cardDotRef} className="component-callout__dot component-callout__dot--card" r="3" />
      </svg>

      <aside
        ref={cardRef}
        className="component-card"
        aria-live="polite"
        data-testid="component-detail"
      >
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
    </div>
  );
}
