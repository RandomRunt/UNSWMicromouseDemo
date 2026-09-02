import type { ComponentId } from '../types/showcase';

export const senseChapterAssembledComponentIds = [
  'microcontroller',
  'oled_display',
  'ball_caster_front',
  'ball_caster_rear',
] as const satisfies readonly ComponentId[];

const SENSE_CHAPTER_ASSEMBLED_COMPONENTS = new Set<ComponentId>(
  senseChapterAssembledComponentIds,
);

export function getExplodeAmount(activeChapter: number) {
  // Chapter 03 keeps the chapter 02 teardown in place so the sensing hardware
  // remains separated and legible while its measurements are introduced.
  return activeChapter === 1 || activeChapter === 2 ? 1 : 0;
}

export function getComponentExplodeAmount(activeChapter: number, componentId: ComponentId) {
  if (activeChapter === 2 && SENSE_CHAPTER_ASSEMBLED_COMPONENTS.has(componentId)) return 0;
  return getExplodeAmount(activeChapter);
}

const STORY_CHAPTER_COUNT = 6;
const STORY_SCROLL_INTERVAL_COUNT = STORY_CHAPTER_COUNT - 1;
const MOVE_CHAPTER_INDEX = 4;
const MAZE_VISIBLE_START = MOVE_CHAPTER_INDEX / STORY_CHAPTER_COUNT;
const MAZE_COLLAPSE_START = MOVE_CHAPTER_INDEX / STORY_SCROLL_INTERVAL_COUNT;
const MAZE_COLLAPSE_END = (MOVE_CHAPTER_INDEX + 0.5) / STORY_SCROLL_INTERVAL_COUNT;
export const MIN_SCENE_SCALE = 0.001;

export function getMazeScrollScale(progress: number) {
  if (progress < MAZE_VISIBLE_START || progress >= MAZE_COLLAPSE_END) {
    return MIN_SCENE_SCALE;
  }
  if (progress <= MAZE_COLLAPSE_START) return 1;

  const collapseProgress = Math.min(
    1,
    Math.max(
      0,
      (progress - MAZE_COLLAPSE_START) / (MAZE_COLLAPSE_END - MAZE_COLLAPSE_START),
    ),
  );
  const easedCollapse = collapseProgress * collapseProgress * (3 - 2 * collapseProgress);
  return Math.max(MIN_SCENE_SCALE, 1 - easedCollapse);
}
