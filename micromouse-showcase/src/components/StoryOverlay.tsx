import { componentDefinitions } from '../config/components';
import { chapters } from '../story/chapters';
import type { ComponentId } from '../types/showcase';
import type { ThemeMode } from '../App';

interface StoryOverlayProps {
  activeChapter: number;
  selected: ComponentId | null;
  assetAvailable: boolean;
  onSelect: (id: ComponentId) => void;
  onRestart: () => void;
  theme: ThemeMode;
  onToggleTheme: () => void;
}

export function StoryOverlay({
  activeChapter,
  selected,
  assetAvailable,
  onSelect,
  onRestart,
  theme,
  onToggleTheme,
}: StoryOverlayProps) {
  return (
    <div className="story-overlay">
      <header className="topbar">
        <a className="brand" href="#chapter-meet" aria-label="Micromouse showcase home">
          <span className="brand__mark" aria-hidden="true">M</span>
          <span>
            <strong>MICROMOUSE</strong>
            <small>UNSW // DEMO DAY</small>
          </span>
        </a>

        <div className="topbar__status" aria-label="Showcase system status">
          <span className="status-dot" aria-hidden="true" />
          <span>{assetAvailable ? 'DIGITAL TWIN ONLINE' : 'PROCEDURAL TWIN ONLINE'}</span>
        </div>

        <div className="topbar__actions">
          <button
            className="theme-button"
            type="button"
            onClick={onToggleTheme}
            aria-label={`Switch to ${theme === 'dark' ? 'light' : 'dark'} mode`}
          >
            <span className="theme-button__track" aria-hidden="true"><i /></span>
            <span>{theme === 'dark' ? 'DARK' : 'LIGHT'}</span>
          </button>

          <button className="restart-button" type="button" onClick={onRestart}>
          <span aria-hidden="true">↺</span> Restart tour
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

      {activeChapter >= 1 && (
        <div className="component-index">
          <span className="component-index__label">COMPONENT INDEX</span>
          <div className="component-index__grid">
            {componentDefinitions.map((component) => (
              <button
                type="button"
                key={component.id}
                className={selected === component.id ? 'is-selected' : ''}
                onClick={() => onSelect(component.id)}
                aria-pressed={selected === component.id}
              >
                <span style={{ backgroundColor: component.accent }} aria-hidden="true" />
                {component.shortLabel}
              </button>
            ))}
          </div>
        </div>
      )}

      <div className="scroll-cue" aria-hidden="true">
        <span>SCROLL TO DISASSEMBLE</span>
        <i />
      </div>
    </div>
  );
}
