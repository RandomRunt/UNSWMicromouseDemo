import { useEffect, useState } from 'react';

const clamp = (value: number, min = 0, max = 1) => Math.min(max, Math.max(min, value));

export function useScrollProgress(chapterCount: number) {
  const [progress, setProgress] = useState(0);
  const [activeChapter, setActiveChapter] = useState(0);
  const [isAtBottom, setIsAtBottom] = useState(false);

  useEffect(() => {
    let frame = 0;

    const update = () => {
      frame = 0;
      const maxScroll = Math.max(1, document.documentElement.scrollHeight - window.innerHeight);
      const nextProgress = clamp(window.scrollY / maxScroll);
      setProgress(nextProgress);
      setActiveChapter(Math.min(chapterCount - 1, Math.floor(nextProgress * chapterCount)));
      setIsAtBottom(maxScroll - window.scrollY <= 2);
    };

    const onScroll = () => {
      if (!frame) frame = window.requestAnimationFrame(update);
    };

    update();
    window.addEventListener('scroll', onScroll, { passive: true });
    window.addEventListener('resize', onScroll);

    return () => {
      window.removeEventListener('scroll', onScroll);
      window.removeEventListener('resize', onScroll);
      if (frame) window.cancelAnimationFrame(frame);
    };
  }, [chapterCount]);

  return { progress, activeChapter, isAtBottom };
}
