import { describe, expect, it } from 'vitest';
import {
  componentDefinitions,
  requiredMeshNames,
  sensingComponentIds,
} from '../src/config/components';
import {
  getComponentExplodeAmount,
  getExplodeAmount,
  getMazeScrollScale,
  MIN_SCENE_SCALE,
  senseChapterAssembledComponentIds,
  shouldSpinWheels,
} from '../src/scene/motion';
import {
  getResponsiveMazeScale,
  getResponsiveRobotScale,
  MOBILE_MAZE_SCALE,
  MOBILE_ROBOT_SCALE,
  MOBILE_VIEW_BREAKPOINT,
} from '../src/config/responsive';

describe('Micromouse model contract', () => {
  it('uses unique lowercase ASCII mesh names', () => {
    const names = requiredMeshNames;
    expect(new Set(names).size).toBe(names.length);
    names.forEach((name) => expect(name).toMatch(/^[a-z0-9_]+$/));
  });

  it('defines presentation behavior for every interactive component', () => {
    expect(componentDefinitions).toHaveLength(20);
    componentDefinitions.forEach((component) => {
      expect(component.title.length).toBeGreaterThan(2);
      expect(component.description.length).toBeGreaterThan(24);
      expect(component.explodeOffset).toHaveLength(3);
    });
  });

  it('keeps the chapter 02 explosion through the sensing chapter', () => {
    expect([0, 1, 2, 3, 4, 5].map(getExplodeAmount)).toEqual([0, 1, 1, 0, 0, 0]);
  });

  it('reassembles non-featured parts and all ToF sensors during chapter 03', () => {
    expect(senseChapterAssembledComponentIds).toEqual([
      'microcontroller',
      'oled_display',
      'ball_caster_front',
      'ball_caster_rear',
      'tof_left',
      'tof_front',
      'tof_right',
    ]);
    senseChapterAssembledComponentIds.forEach((id) => {
      expect(getComponentExplodeAmount(1, id)).toBe(1);
      expect(getComponentExplodeAmount(2, id)).toBe(0);
    });
    expect(getComponentExplodeAmount(2, 'tof_front')).toBe(0);
  });

  it('highlights all range and motion sensors during chapter 03', () => {
    expect(sensingComponentIds).toEqual([
      'tof_left',
      'tof_front',
      'tof_right',
      'imu',
      'encoder_left',
      'encoder_right',
    ]);
  });

  it('spins the wheels only during chapter 04 when motion is allowed', () => {
    expect([0, 1, 2, 3, 4, 5].map((chapter) => shouldSpinWheels(chapter, false))).toEqual([
      false,
      false,
      false,
      true,
      false,
      false,
    ]);
    expect(shouldSpinWheels(3, true)).toBe(false);
  });

  it('uses one responsive scale for the mobile robot and its sensor beams', () => {
    expect(getResponsiveRobotScale(MOBILE_VIEW_BREAKPOINT)).toBe(MOBILE_ROBOT_SCALE);
    expect(getResponsiveRobotScale(MOBILE_VIEW_BREAKPOINT + 1)).toBe(1);
  });

  it('renders the mobile maze at 60 percent of its desktop size', () => {
    expect(getResponsiveMazeScale(MOBILE_VIEW_BREAKPOINT)).toBe(MOBILE_MAZE_SCALE);
    expect(getResponsiveMazeScale(MOBILE_VIEW_BREAKPOINT + 1)).toBe(1);
  });

  it('finishes shrinking the maze halfway through the 05 to 06 transition', () => {
    expect(getMazeScrollScale(4 / 6)).toBe(1);
    expect(getMazeScrollScale(4 / 5)).toBe(1);
    expect(getMazeScrollScale(4.25 / 5)).toBeGreaterThan(MIN_SCENE_SCALE);
    expect(getMazeScrollScale(4.25 / 5)).toBeLessThan(1);
    expect(getMazeScrollScale(4.5 / 5)).toBe(MIN_SCENE_SCALE);
    expect(getMazeScrollScale(1)).toBe(MIN_SCENE_SCALE);
  });
});
