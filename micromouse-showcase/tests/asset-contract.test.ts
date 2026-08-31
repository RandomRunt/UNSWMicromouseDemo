import { describe, expect, it } from 'vitest';
import { componentDefinitions, requiredMeshNames } from '../src/config/components';
import { getExplodeAmount } from '../src/scene/motion';

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

  it('only explodes automatically during chapter 02', () => {
    expect([0, 1, 2, 3, 4, 5].map(getExplodeAmount)).toEqual([0, 1, 0, 0, 0, 0]);
  });
});
