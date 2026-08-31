export function getExplodeAmount(activeChapter: number) {
  // Chapter 02 is the dedicated teardown view. Every other chapter stays
  // assembled unless ShowcaseCanvas deliberately asks for this chapter's
  // amount (the manual Explode button in chapter 06 does exactly that).
  return activeChapter === 1 ? 1 : 0;
}
