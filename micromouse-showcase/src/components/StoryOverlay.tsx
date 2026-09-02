import { useEffect, useState } from 'react';
import { componentDefinitions } from '../config/components';
import { chapters } from '../story/chapters';
import type { ComponentId } from '../types/showcase';
import type { ThemeMode } from '../App';

interface StoryOverlayProps {
  activeChapter: number;
  explorationEnabled: boolean;
  mobile3DViewActive: boolean;
  onToggleMobile3DView: () => void;
  hasInspectedComponent: boolean;
  hasUsedOrbitControls: boolean;
  selected: ComponentId | null;
  assetAvailable: boolean;
  onSelect: (id: ComponentId) => void;
  explorationExploded: boolean;
  onToggleExploded: () => void;
  theme: ThemeMode;
  onToggleTheme: () => void;
  isAutoScrolling: boolean;
  onToggleAutoScroll: () => void;
}

export function StoryOverlay({
  activeChapter,
  explorationEnabled,
  mobile3DViewActive,
  onToggleMobile3DView,
  hasInspectedComponent,
  hasUsedOrbitControls,
  selected,
  assetAvailable,
  onSelect,
  explorationExploded,
  onToggleExploded,
  theme,
  onToggleTheme,
  isAutoScrolling,
  onToggleAutoScroll,
}: StoryOverlayProps) {
  const isInsideChapter = activeChapter === 1;
  const isExploreChapter = activeChapter === chapters.length - 1;
  const [isComponentIndexOpen, setIsComponentIndexOpen] = useState(false);

  useEffect(() => {
    setIsComponentIndexOpen(false);
  }, [activeChapter]);

  const selectIndexedComponent = (id: ComponentId) => {
    onSelect(id);
    if (window.matchMedia('(max-width: 640px)').matches) {
      setIsComponentIndexOpen(false);
    }
  };

  return (
    <div className="story-overlay">
      <header className="topbar">
        <a className="brand" href="#chapter-meet" aria-label="Micromouse showcase home">
          <span className="brand__mark" aria-hidden="true">M</span>
          <span>
            <strong>MICROMOUSE</strong>
            <small>UNSW // ROBOTICS</small>
          </span>
        </a>

        <div className="topbar__status" aria-label="Showcase system status">
          <span className="status-dot" aria-hidden="true" />
          <span>{assetAvailable ? 'DIGITAL TWIN ONLINE' : 'MICROMOUSE VISUALISATION'}</span>
        </div>

        <div className="topbar__actions">
          <button
            className="auto-scroll-button"
            type="button"
            onClick={onToggleAutoScroll}
            aria-pressed={isAutoScrolling}
            aria-label={isAutoScrolling ? 'Stop auto scroll' : 'Start auto scroll'}
          >
            <span aria-hidden="true">{isAutoScrolling ? '■' : '▶'}</span>
            <span>{isAutoScrolling ? 'Stop' : 'Auto scroll'}</span>
          </button>

          <button
            className="theme-button"
            type="button"
            onClick={onToggleTheme}
            aria-label={`Switch to ${theme === 'dark' ? 'light' : 'dark'} mode`}
          >
            <span className="theme-button__track" aria-hidden="true"><i /></span>
            <span>{theme === 'dark' ? 'DARK' : 'LIGHT'}</span>
          </button>

        </div>
      </header>

      <nav className="chapter-rail" aria-label="Story chapters">
        <ol>
          {chapters.map((chapter, index) => (
            <li key={chapter.id} className={index === activeChapter ? 'is-active' : ''}>
              <a href={`#chapter-${chapter.id}`} aria-current={index === activeChapter ? 'step' : undefined}>
                <span>{chapter.number}</span>
                <strong>{chapter.id}</strong>
              </a>
            </li>
          ))}
        </ol>
      </nav>

      {activeChapter === 3 && (
        <div className="signal-chain" aria-label="Micromouse control pipeline">
          {['SENSE', 'ESTIMATE', 'FOLLOW', 'DRIVE'].map((stage, index) => (
            <div className="signal-chain__stage" key={stage}>
              <span>{String(index + 1).padStart(2, '0')}</span>
              <strong>{stage}</strong>
              {index < 3 && <i aria-hidden="true" />}
            </div>
          ))}
        </div>
      )}

      {activeChapter === chapters.length - 1 && (
        <>
          <button
            className="explode-toggle"
            type="button"
            onClick={onToggleExploded}
            aria-pressed={explorationExploded}
          >
            <span aria-hidden="true">06 //</span>
            {explorationExploded ? 'Assemble robot' : 'Explode robot'}
          </button>

          <button
            className="mobile-3d-view-toggle"
            type="button"
            onClick={onToggleMobile3DView}
            aria-pressed={mobile3DViewActive}
            data-testid="mobile-3d-view-toggle"
          >
            <span className="mobile-3d-view-toggle__icon" aria-hidden="true"><i /></span>
            <span>{mobile3DViewActive ? 'Exit 3D view' : 'Enter 3D view'}</span>
          </button>
        </>
      )}

      {isInsideChapter && !hasInspectedComponent && !isComponentIndexOpen && (
        <div
          className="interaction-hint interaction-hint--components"
          role="status"
          data-testid="component-hint"
        >
          <span className="interaction-hint__eyebrow">
            <i aria-hidden="true" />
            02 // Inspection ready
          </span>
          <div className="interaction-hint__instruction">
            <svg viewBox="0 0 88 64" aria-hidden="true">
              <rect className="interaction-hint__component" x="10" y="11" width="25" height="17" rx="2" />
              <rect className="interaction-hint__component" x="48" y="9" width="27" height="19" rx="2" />
              <rect className="interaction-hint__component interaction-hint__component--active" x="18" y="39" width="29" height="16" rx="2" />
              <path className="interaction-hint__cursor" d="m55 34 1 21 5-5 4 9 5-2-4-9 8-1Z" />
            </svg>
            <span>
              <strong>Select a component</strong>
              <small>Click the robot or use the index</small>
            </span>
          </div>
        </div>
      )}

      {isExploreChapter && explorationEnabled && !hasUsedOrbitControls && (
        <div
          className="interaction-hint interaction-hint--orbit"
          role="status"
          data-testid="orbit-hint"
        >
          <span className="interaction-hint__eyebrow">
            <i aria-hidden="true" />
            360 // Controls online
          </span>
          <div className="interaction-hint__instruction">
            <svg viewBox="0 0 88 64" aria-hidden="true">
              <path className="interaction-hint__arc" d="M14 38C20 14 62 9 76 30" />
              <path className="interaction-hint__arrow" d="m68 25 8 5-7 5" />
              <path className="interaction-hint__arc interaction-hint__arc--return" d="M74 43C59 58 28 56 15 38" />
              <path className="interaction-hint__arrow" d="m22 45-7-7 9-2" />
              <rect className="interaction-hint__mouse" x="37" y="20" width="15" height="25" rx="7.5" />
              <path className="interaction-hint__mouse-line" d="M44.5 20v8" />
            </svg>
            <span>
              <strong>Drag to orbit</strong>
              <small>Scroll or pinch to zoom</small>
            </span>
          </div>
        </div>
      )}

      {activeChapter >= 1 && (
        <div className={`component-index${isComponentIndexOpen ? ' is-open' : ''}`}>
          <button
            className="component-index__toggle"
            type="button"
            onClick={() => setIsComponentIndexOpen((current) => !current)}
            aria-expanded={isComponentIndexOpen}
            aria-controls="component-index-panel"
          >
            <span className="component-index__label">COMPONENT INDEX</span>
            <span className="component-index__count">{componentDefinitions.length}</span>
            <span className="component-index__toggle-icon" aria-hidden="true" />
          </button>
          <div
            id="component-index-panel"
            className="component-index__panel"
            hidden={!isComponentIndexOpen}
          >
            <div className="component-index__grid">
              {componentDefinitions.map((component) => (
                <button
                  type="button"
                  key={component.id}
                  className={`component-index__item${selected === component.id ? ' is-selected' : ''}`}
                  data-component-trigger
                  onClick={() => selectIndexedComponent(component.id)}
                  aria-pressed={selected === component.id}
                >
                  <span style={{ backgroundColor: component.accent }} aria-hidden="true" />
                  {component.shortLabel}
                </button>
              ))}
            </div>
          </div>
        </div>
      )}

      <div className="scroll-cue" aria-hidden="true">
        <span>SCROLL TO NAVIGATE</span>
        <i />
      </div>
    </div>
  );
}
