import { ArrowRight, Box, Expand, LogOut, Orbit, RotateCcw } from 'lucide-react';
import { useEffect, useState } from 'react';
import type { ThemeMode } from '../App';
import { componentDefinitions } from '../config/components';
import { chapters } from '../story/chapters';
import type { ComponentId, ModelAvailability } from '../types/showcase';

interface StoryOverlayProps {
  activeChapter: number;
  explorationEnabled: boolean;
  isMobileViewport: boolean;
  mobile3DViewActive: boolean;
  showMobile3DViewGlow: boolean;
  onToggleMobile3DView: () => void;
  onResetView: () => void;
  hasInspectedComponent: boolean;
  hasUsedOrbitControls: boolean;
  selected: ComponentId | null;
  modelAvailability: ModelAvailability;
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
  isMobileViewport,
  mobile3DViewActive,
  showMobile3DViewGlow,
  onToggleMobile3DView,
  onResetView,
  hasInspectedComponent,
  hasUsedOrbitControls,
  selected,
  modelAvailability,
  onSelect,
  explorationExploded,
  onToggleExploded,
  theme,
  onToggleTheme,
  isAutoScrolling,
  onToggleAutoScroll,
}: StoryOverlayProps) {
  const isExploreChapter = activeChapter === chapters.length - 1;
  const systemStatus = modelAvailability === 'checking'
    ? 'LOADING DIGITAL TWIN'
    : modelAvailability === 'available'
      ? 'DIGITAL TWIN ONLINE'
      : 'MICROMOUSE VISUALISATION';
  const [isComponentIndexOpen, setIsComponentIndexOpen] = useState(false);
  const [showControlLoop, setShowControlLoop] = useState(activeChapter === 3);
  const [isControlLoopExiting, setIsControlLoopExiting] = useState(false);

  useEffect(() => {
    setIsComponentIndexOpen(false);
  }, [activeChapter]);

  useEffect(() => {
    if (activeChapter === 3) {
      setShowControlLoop(true);
      setIsControlLoopExiting(false);
      return;
    }

    if (!showControlLoop) return;

    setIsControlLoopExiting(true);
  }, [activeChapter, showControlLoop]);

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
          <span>{systemStatus}</span>
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

      {showControlLoop && (
        <div
          className={`control-loop${isControlLoopExiting ? ' control-loop--exiting' : ''}`}
          aria-label="Micromouse control loop: sense, think, move"
          aria-hidden={isControlLoopExiting || undefined}
          onAnimationEnd={(event) => {
            if (
              event.target !== event.currentTarget
              || event.animationName !== 'panel-out'
              || !isControlLoopExiting
            ) return;
            setShowControlLoop(false);
            setIsControlLoopExiting(false);
          }}
        >
          <div className="control-loop__header" aria-hidden="true">
            <span className="control-loop__label"><i />CONTROL LOOP</span>
            {/* <small>04 // DECISION</small> */}
          </div>
          <ol>
            {[
              { number: '01', name: 'SENSE' },
              { number: '02', name: 'THINK' },
              { number: '03', name: 'MOVE' },
            ].map((stage) => (
              <li key={stage.name}>
                <small aria-hidden="true">{stage.number}</small>
                <strong>{stage.name}</strong>
                {stage.number !== '03' && (
                  <ArrowRight className="control-loop__stage-arrow" aria-hidden="true" />
                )}
              </li>
            ))}
          </ol>
          <div className="control-loop__feedback" aria-hidden="true">
            <div className="control-loop__feedback-path">
              <svg viewBox="0 0 100 34" preserveAspectRatio="none">
                <path
                  className="control-loop__feedback-shaft"
                  d="M100 0 V24 A2.5 9 0 0 1 97.5 33 H2.5 A2.5 9 0 0 1 0 24 V0"
                />
              </svg>
              <svg className="control-loop__feedback-arrowhead" viewBox="0 0 14 7">
                <path d="M1 6 L7 0 L13 6" />
              </svg>
              <span>FEEDBACK</span>
            </div>
          </div>
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
            <Expand className="explode-toggle__icon" aria-hidden="true" />
            {explorationExploded ? 'Assemble robot' : 'Explode robot'}
          </button>

          <button
            className="mobile-3d-view-toggle"
            type="button"
            onClick={onToggleMobile3DView}
            aria-pressed={mobile3DViewActive}
            data-attention-glow={showMobile3DViewGlow}
            data-testid="mobile-3d-view-toggle"
          >
            {mobile3DViewActive
              ? <LogOut className="mobile-3d-view-toggle__icon" aria-hidden="true" />
              : <Box className="mobile-3d-view-toggle__icon" aria-hidden="true" />}
            <span>{mobile3DViewActive ? 'Exit 3D view' : 'Enter 3D view'}</span>
          </button>

          {mobile3DViewActive && (
            <button
              className="mobile-reset-view"
              type="button"
              onClick={onResetView}
              aria-label="Reset 3D view"
              title="Reset view"
            >
              <RotateCcw aria-hidden="true" />
            </button>
          )}
        </>
      )}

      {isExploreChapter
        && explorationEnabled
        && hasUsedOrbitControls
        && !hasInspectedComponent
        && !isComponentIndexOpen && (
        <div
          className="interaction-hint interaction-hint--components interaction-hint--components-delayed"
          role="status"
          data-testid="component-hint"
        >
          <span className="interaction-hint__eyebrow">
            <i aria-hidden="true" />
            06 // Inspection ready
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
              <small>{isMobileViewport ? 'Tap the robot or use the index' : 'Click the robot or use the index'}</small>
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
            <Orbit className="interaction-hint__orbit-icon" aria-hidden="true" />
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
            data-testid="component-index-panel"
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

      {!isExploreChapter && (
        <div
          className={`scroll-cue${activeChapter === 0 ? ' scroll-cue--first-chapter' : ''}`}
          aria-hidden="true"
        >
          <span>SCROLL TO NAVIGATE</span>
          <i />
        </div>
      )}
    </div>
  );
}
