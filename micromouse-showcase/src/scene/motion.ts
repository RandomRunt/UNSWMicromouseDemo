export function getExplodeAmount(activeChapter: number) {
  if (activeChapter === 1) return 1;
  if (activeChapter === 2) return 0.58;
  if (activeChapter === 3) return 0.22;
  if (activeChapter === 5) return 0.38;
  return 0;
}
