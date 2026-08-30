import { describe, expect, it } from 'vitest';
import { componentDefinitions, requiredMeshNames } from '../src/config/components';

describe('Micromouse model contract', () => {
  it('uses unique lowercase ASCII mesh names', () => {
    const names = requiredMeshNames;
    expect(new Set(names).size).toBe(names.length);
    names.forEach((name) => expect(name).toMatch(/^[a-z0-9_]+$/));
  });

  it('defines presentation behavior for every interactive component', () => {
    expect(componentDefinitions).toHaveLength(14);
    componentDefinitions.forEach((component) => {
      expect(component.title.length).toBeGreaterThan(2);
      expect(component.description.length).toBeGreaterThan(24);
      expect(component.explodeOffset).toHaveLength(3);
    });
  });
});
