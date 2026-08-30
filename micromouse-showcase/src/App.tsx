import { useCallback, useEffect, useState } from 'react';
import { ComponentLabel } from './components/ComponentLabel';
import { ShowcaseCanvas } from './components/ShowcaseCanvas';
import { StoryOverlay } from './components/StoryOverlay';
import { chapters } from './story/chapters';
import { useKioskReset } from './story/useKioskReset';
import { useReducedMotion } from './story/useReducedMotion';
import { useScrollProgress } from './story/useScrollProgress';
import type { ComponentId } from './types/showcase';

export type ThemeMode = 'light' | 'dark';

function App() {
  const { progress, activeChapter, isAtBottom } = useScrollProgress(chapters.length);
  const reducedMotion = useReducedMotion();
  const [selected, setSelected] = useState<ComponentId | null>(null);
  const [assetAvailable, setAssetAvailable] = useState(false);
  const [theme, setTheme] = useState<ThemeMode>(() => {
    const savedTheme = window.localStorage.getItem('micromouse-theme');
    if (savedTheme === 'light' || savedTheme === 'dark') return savedTheme;
    return window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  });

  useEffect(() => {
    document.documentElement.dataset.theme = theme;
    document.documentElement.style.colorScheme = theme;
    window.localStorage.setItem('micromouse-theme', theme);
  }, [theme]);

  useEffect(() => {
    const controller = new AbortController();
    fetch('/models/micromouse.glb', { method: 'HEAD', cache: 'no-store', signal: controller.signal })
      .then((response) => {
        const type = response.headers.get('content-type') ?? '';
        setAssetAvailable(response.ok && !type.includes('text/html'));
      })
      .catch(() => setAssetAvailable(false));
    return () => controller.abort();
  }, []);

  const restartTour = useCallback(() => {
    setSelected(null);
    window.scrollTo({ top: 0, behavior: reducedMotion ? 'auto' : 'smooth' });
  }, [reducedMotion]);

  useKioskReset(restartTour);

  return (
    <>
      <ShowcaseCanvas
        progress={progress}
        activeChapter={activeChapter}
        explorationEnabled={isAtBottom}
        selected={selected}
        onSelect={setSelected}
        reducedMotion={reducedMotion}
        assetAvailable={assetAvailable}
        theme={theme}
      />

      <StoryOverlay
        activeChapter={activeChapter}
        selected={selected}
        assetAvailable={assetAvailable}
        onSelect={setSelected}
        onRestart={restartTour}
        theme={theme}
        onToggleTheme={() => setTheme((current) => current === 'dark' ? 'light' : 'dark')}
      />

      <ComponentLabel selected={selected} onClose={() => setSelected(null)} />

      <main id="story" className="story" tabIndex={-1}>
        {chapters.map((chapter, index) => (
          <section
            id={`chapter-${chapter.id}`}
            className={`story-chapter story-chapter--${chapter.id}`}
            key={chapter.id}
            aria-labelledby={`chapter-title-${chapter.id}`}
            data-testid={`chapter-${chapter.id}`}
          >
            <div className="chapter-copy">
              <div className="chapter-copy__meta">
                <span>{chapter.number} / {String(chapters.length).padStart(2, '0')}</span>
                <span>{chapter.eyebrow}</span>
              </div>
              <h1 id={`chapter-title-${chapter.id}`}>{chapter.title}</h1>
              <p>{chapter.body}</p>
              <div className="chapter-metric">
                <strong>{chapter.metric}</strong>
                <span>{chapter.metricLabel}</span>
              </div>
              {index === 0 && (
                <p className="chapter-note">
                  <span aria-hidden="true">●</span> Scroll to begin the teardown
                </p>
              )}
            </div>
          </section>
        ))}
      </main>

      <div className="sr-only" aria-live="polite">
        Chapter {activeChapter + 1} of {chapters.length}: {chapters[activeChapter].title}
      </div>
    </>
  );
}

export default App;
