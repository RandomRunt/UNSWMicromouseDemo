import AxeBuilder from '@axe-core/playwright';
import { expect, test, type Page } from '@playwright/test';

// Keep the real GLB route in the complete-story test. DOM and input journeys
// use the supported fallback so CI does not decode the 32 MB model per test.
async function routeModelToProceduralFallback(page: Page) {
  await page.route('**/models/micromouse.glb', async (route) => {
    if (route.request().method() === 'HEAD') {
      await route.fulfill({ status: 404 });
      return;
    }

    await route.continue();
  });
}

test('tells the complete six-chapter story without console errors', async ({ page }) => {
  const errors: string[] = [];
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });

  await page.goto('/');
  const openingHeading = page.getByTestId('chapter-meet').getByRole('heading');
  await expect(openingHeading).toBeVisible();
  await expect(openingHeading).toHaveText(/\S+/);
  await expect(page.getByTestId('showcase-canvas')).toBeVisible();
  await expect(page.locator('script[data-cf-beacon]')).toHaveCount(0);

  for (const id of ['meet', 'inside', 'sense', 'think', 'move', 'explore']) {
    await expect(page.getByTestId(`chapter-${id}`)).toBeAttached();
  }

  expect(errors).toEqual([]);
});

test('keeps the loader visible until a missing model selects the fallback', async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== 'chromium', 'Model startup state only needs one browser project');

  let resolveModelCheck: () => void = () => undefined;
  const modelCheckPending = new Promise<void>((resolve) => {
    resolveModelCheck = resolve;
  });

  await page.route('**/models/micromouse.glb', async (route) => {
    if (route.request().method() !== 'HEAD') {
      await route.continue();
      return;
    }

    await modelCheckPending;
    await route.fulfill({ status: 404 });
  });

  await page.goto('/');
  const canvas = page.getByTestId('showcase-canvas');
  await expect(canvas).toHaveAttribute('data-model-source', 'loading');
  await expect(page.getByTestId('scene-loader')).toContainText('LOADING MODEL...');
  await expect(page.getByLabel('Showcase system status')).toContainText('LOADING DIGITAL TWIN');

  resolveModelCheck();
  await expect(canvas).toHaveAttribute('data-model-source', 'procedural-fallback');
  await expect(page.getByTestId('scene-loader')).toHaveCount(0);
  await expect(page.getByLabel('Showcase system status')).toContainText('MICROMOUSE VISUALISATION');
});

test('supports keyboard-accessible component inspection', async ({ page }) => {
  await routeModelToProceduralFallback(page);
  await page.goto('/');
  await page.getByTestId('chapter-inside').scrollIntoViewIfNeeded();
  const canvas = page.getByTestId('showcase-canvas');
  await expect(canvas).toHaveAttribute('data-inspection-guides', 'false');

  const indexToggle = page.getByRole('button', { name: /component index/i });
  const indexPanel = page.getByTestId('component-index-panel');
  await expect(indexToggle).toHaveAttribute('aria-expanded', 'false');
  await expect(indexPanel).toHaveAttribute('hidden', '');
  await expect(page.getByTestId('component-hint')).toHaveCount(0);
  await indexToggle.click();
  await expect(indexToggle).toHaveAttribute('aria-expanded', 'true');
  await expect(indexPanel).not.toHaveAttribute('hidden', '');
  await expect(page.getByTestId('component-hint')).toHaveCount(0);

  const controllerButton = page.getByRole('button', { name: 'MCU', includeHidden: true });
  await expect(controllerButton).toBeVisible();
  await controllerButton.focus();
  await page.keyboard.press('Enter');

  const componentDetail = page.getByTestId('component-detail');
  await expect(componentDetail).toContainText('Microcontroller');
  await expect(componentDetail.getByLabel('Part name')).toContainText('Arduino Nano');
  await expect(controllerButton).toHaveAttribute('aria-pressed', 'true');
  await expect(page.getByTestId('component-hint')).toHaveCount(0);

  if (test.info().project.name === 'mobile-chromium') {
    await expect(indexToggle).toHaveAttribute('aria-expanded', 'false');
    await expect(indexPanel).toHaveAttribute('hidden', '');
  } else {
    await indexToggle.click();
    await expect(indexToggle).toHaveAttribute('aria-expanded', 'false');
    await expect(indexPanel).toHaveAttribute('hidden', '');
  }

  await page.getByTestId('chapter-sense').scrollIntoViewIfNeeded();
  await expect(canvas).toHaveAttribute('data-inspection-guides', 'true');
  await expect(page.locator('[data-axis-direction="x"]')).toHaveText('Y');
  await expect(page.locator('[data-axis-direction="y"]')).toHaveText('Z');
  await expect(page.locator('[data-axis-direction="z"]')).toHaveText('X');
  await expect(page.getByText('ENCODER ROTATION')).toHaveCount(0);

  await page.getByTestId('chapter-think').scrollIntoViewIfNeeded();
  await expect(page.getByLabel(/Micromouse control loop/)).toContainText(/SENSE.*THINK.*MOVE/);
  await expect(canvas).toHaveAttribute('data-wheel-motion', 'true');
  await expect(page.locator('.inspection-axis-label')).toHaveCount(0);

  await page.getByTestId('chapter-explore').scrollIntoViewIfNeeded();
  await expect(page.getByText('SCROLL TO NAVIGATE')).toHaveCount(0);
});

test('enables 360-degree controls at the desktop page end or through mobile 3D view', async ({ page }) => {
  await routeModelToProceduralFallback(page);
  await page.goto('/');
  const canvas = page.getByTestId('showcase-canvas');
  const canvasElement = canvas.locator('canvas');

  await page.evaluate(() => {
    const maxScroll = document.documentElement.scrollHeight - window.innerHeight;
    window.scrollTo(0, maxScroll * 0.9);
  });
  await expect(canvas).toHaveAttribute('data-exploration-enabled', 'false');
  await expect(page.getByTestId('chapter-explore')).toBeInViewport();

  if (test.info().project.name === 'mobile-chromium') {
    await expect(canvasElement).toHaveCSS('touch-action', 'pan-y');
    await expect(page.locator('html')).toHaveCSS('overscroll-behavior-y', 'none');
    const mobile3DViewButton = page.getByRole('button', { name: 'Enter 3D view' });
    await expect(mobile3DViewButton).toBeVisible();
    await expect(mobile3DViewButton).toHaveAttribute('aria-pressed', 'false');
    await mobile3DViewButton.click();

    await expect(canvas).toHaveAttribute('data-exploration-enabled', 'true');
    await expect(page.locator('html')).toHaveAttribute('data-mobile-3d-view', 'true');
    await expect(canvasElement).toHaveCSS('touch-action', 'none');
    await expect(page.getByRole('button', { name: 'Exit 3D view' })).toHaveAttribute(
      'aria-pressed',
      'true',
    );

    await page.getByRole('button', { name: 'Exit 3D view' }).click();
    await expect(canvas).toHaveAttribute('data-exploration-enabled', 'false');
    await expect(page.locator('html')).not.toHaveAttribute('data-mobile-3d-view', 'true');
    await expect(canvasElement).toHaveCSS('touch-action', 'pan-y');
    return;
  }

  await page.evaluate(() => window.scrollTo(0, document.documentElement.scrollHeight));
  await expect(canvas).toHaveAttribute('data-exploration-enabled', 'true');
});

test('allows touch scrolling through the story on mobile', async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== 'mobile-chromium', 'Touch scrolling only applies to mobile');
  await routeModelToProceduralFallback(page);
  await page.goto('/');

  // A normal swipe should land on the scrollable story, not the fixed canvas.
  // This is important on mobile Safari, which may not transfer a gesture that
  // starts on a fixed WebGL canvas to the document scroller.
  await expect(page.locator('.story')).toHaveCSS('pointer-events', 'auto');
  await expect(page.locator('.story')).toHaveCSS('touch-action', 'pan-y');
  await expect(page.locator('.chapter-copy').first()).toHaveCSS('pointer-events', 'auto');
  await expect(page.getByTestId('showcase-canvas')).toHaveCSS('pointer-events', 'none');
  await expect.poll(() => page.evaluate(() => {
    const target = document.elementFromPoint(window.innerWidth / 2, window.innerHeight * 0.7);
    return Boolean(target?.closest('.story'));
  })).toBe(true);

  const client = await page.context().newCDPSession(page);
  await client.send('Input.dispatchTouchEvent', {
    type: 'touchStart',
    touchPoints: [{ x: 195, y: 620, id: 1 }],
  });
  for (const y of [560, 500, 440, 380, 320, 260]) {
    await client.send('Input.dispatchTouchEvent', {
      type: 'touchMove',
      touchPoints: [{ x: 195, y, id: 1 }],
    });
  }
  await client.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });

  await expect.poll(() => page.evaluate(() => window.scrollY)).toBeGreaterThan(100);
});

test('dismisses the orbit controls hint after a drag or inward zoom', async ({ page }) => {
  await routeModelToProceduralFallback(page);
  await page.goto('/');

  await page.evaluate(() => window.scrollTo(0, document.documentElement.scrollHeight));

  if (test.info().project.name === 'mobile-chromium') {
    await page.getByRole('button', { name: 'Enter 3D view' }).click();
  }

  await expect(page.getByTestId('showcase-canvas')).toHaveAttribute('data-exploration-enabled', 'true');
  await expect(page.getByTestId('orbit-hint')).toContainText('Drag to orbit');

  const canvas = page.getByTestId('showcase-canvas').locator('canvas');
  const bounds = await canvas.boundingBox();
  expect(bounds).not.toBeNull();
  if (!bounds) return;

  const centerX = bounds.x + bounds.width * 0.65;
  const centerY = bounds.y + bounds.height * 0.5;
  if (test.info().project.name === 'mobile-chromium') {
    const client = await page.context().newCDPSession(page);
    await client.send('Input.dispatchTouchEvent', {
      type: 'touchStart',
      touchPoints: [{ x: centerX, y: centerY, id: 1 }],
    });
    for (let step = 1; step <= 4; step += 1) {
      await client.send('Input.dispatchTouchEvent', {
        type: 'touchMove',
        touchPoints: [{
          x: centerX + 15 * step,
          y: centerY + 5 * step,
          id: 1,
        }],
      });
    }
    await client.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await client.detach();
  } else {
    await page.mouse.move(centerX, centerY);
    await page.mouse.wheel(0, -300);
  }
  await expect(page.getByTestId('orbit-hint')).toHaveCount(0);
  await expect(page.getByTestId('component-hint')).toContainText('Select a component');
  if (test.info().project.name === 'mobile-chromium') {
    await expect(page.getByTestId('component-hint')).toContainText('Tap the robot or use the index');
  } else {
    await expect(page.getByTestId('component-hint')).toContainText('Click the robot or use the index');
  }
  await expect(page.getByTestId('component-detail')).toHaveCount(0);
});

test('has no automatically detectable critical accessibility violations', async ({ page }) => {
  await routeModelToProceduralFallback(page);
  await page.goto('/');
  const results = await new AxeBuilder({ page }).analyze();
  expect(results.violations.filter((violation) => violation.impact === 'critical')).toEqual([]);
});

test('keeps every chapter and overlay readable on iPhone-sized screens', async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== 'mobile-chromium', 'iPhone layout coverage only runs in the touch project');
  testInfo.setTimeout(60_000);
  await routeModelToProceduralFallback(page);

  for (const viewport of [{ width: 390, height: 844 }, { width: 430, height: 932 }]) {
    await page.setViewportSize(viewport);
    await page.goto('/');

    const layout = await page.evaluate(() => ({
      documentWidth: document.documentElement.scrollWidth,
      viewportWidth: window.innerWidth,
      chapters: [...document.querySelectorAll<HTMLElement>('.story-chapter')].map((chapter) => {
        const copy = chapter.querySelector<HTMLElement>('.chapter-copy');
        const heading = chapter.querySelector<HTMLElement>('h1');
        const paragraph = chapter.querySelector<HTMLElement>('.chapter-copy > p');
        const metric = chapter.querySelector<HTMLElement>('.chapter-metric');
        if (!copy || !heading || !paragraph || !metric) throw new Error('Chapter copy is incomplete');
        const copyBounds = copy.getBoundingClientRect();
        const chapterBounds = chapter.getBoundingClientRect();
        const headingStyles = getComputedStyle(heading);
        const paragraphStyles = getComputedStyle(paragraph);
        return {
          copyLeft: copyBounds.left,
          copyRight: copyBounds.right,
          copyBottomWithinChapter: copyBounds.bottom <= chapterBounds.bottom,
          headingFits: heading.scrollWidth <= heading.clientWidth,
          paragraphFits: paragraph.scrollWidth <= paragraph.clientWidth,
          headingFontSize: Number.parseFloat(headingStyles.fontSize),
          paragraphFontSize: Number.parseFloat(paragraphStyles.fontSize),
          paragraphLineHeight: Number.parseFloat(paragraphStyles.lineHeight),
          metricDisplay: getComputedStyle(metric).display,
        };
      }),
    }));

    expect(layout.documentWidth).toBeLessThanOrEqual(layout.viewportWidth);
    for (const chapter of layout.chapters) {
      expect(chapter.copyLeft).toBeGreaterThanOrEqual(0);
      expect(chapter.copyRight).toBeLessThanOrEqual(layout.viewportWidth);
      expect(chapter.copyBottomWithinChapter).toBe(true);
      expect(chapter.headingFits).toBe(true);
      expect(chapter.paragraphFits).toBe(true);
      expect(chapter.headingFontSize).toBeGreaterThanOrEqual(36);
      expect(chapter.headingFontSize).toBeLessThan(49);
      expect(chapter.paragraphFontSize).toBeGreaterThanOrEqual(14);
      expect(chapter.paragraphLineHeight).toBeGreaterThanOrEqual(22);
      expect(chapter.metricDisplay).toBe('none');
    }
  }

  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto('/');
  await page.evaluate(() => {
    document.querySelector<HTMLElement>('[data-testid="chapter-inside"]')
      ?.scrollIntoView({ behavior: 'instant' });
  });

  const indexToggle = page.getByRole('button', { name: /component index/i });
  await indexToggle.click();
  const indexBounds = await page.locator('.component-index').boundingBox();
  const railBounds = await page.getByRole('navigation', { name: 'Story chapters' }).boundingBox();
  const topbarBeforeSelectionBounds = await page.locator('.topbar').boundingBox();
  expect(indexBounds).not.toBeNull();
  expect(railBounds).not.toBeNull();
  expect(topbarBeforeSelectionBounds).not.toBeNull();
  if (indexBounds && railBounds && topbarBeforeSelectionBounds) {
    expect(indexBounds.x).toBeGreaterThanOrEqual(0);
    expect(indexBounds.x).toBeLessThanOrEqual(20);
    expect(indexBounds.x + indexBounds.width).toBeLessThanOrEqual(390);
    expect(indexBounds.y).toBeGreaterThanOrEqual(
      topbarBeforeSelectionBounds.y + topbarBeforeSelectionBounds.height,
    );
    expect(indexBounds.y).toBeLessThanOrEqual(
      topbarBeforeSelectionBounds.y + topbarBeforeSelectionBounds.height + 20,
    );
    expect(indexBounds.y + indexBounds.height).toBeLessThanOrEqual(railBounds.y);
  }

  await page.getByRole('button', { name: 'MCU' }).click();
  await expect(indexToggle).toHaveAttribute('aria-expanded', 'false');
  const detail = page.getByTestId('component-detail');
  await expect(detail).toHaveAttribute('data-positioned', 'true');
  await expect(detail.getByLabel('Part name')).toContainText('Arduino Nano');
  const mobileDetailTextSize = await detail.locator('p').evaluate(
    (paragraph) => Number.parseFloat(getComputedStyle(paragraph).fontSize),
  );
  expect(mobileDetailTextSize).toBeLessThan(14);
  const detailBounds = await detail.boundingBox();
  const topbarBounds = await page.locator('.topbar').boundingBox();
  expect(detailBounds).not.toBeNull();
  expect(topbarBounds).not.toBeNull();
  if (detailBounds && topbarBounds && railBounds) {
    expect(detailBounds.y).toBeGreaterThanOrEqual(topbarBounds.y + topbarBounds.height);
    expect(detailBounds.y + detailBounds.height).toBeLessThanOrEqual(railBounds.y);
  }

  await page.evaluate(() => window.scrollTo(0, document.documentElement.scrollHeight));
  await page.getByRole('button', { name: 'Enter 3D view' }).click();
  const orbitHint = page.getByTestId('orbit-hint');
  await expect(orbitHint).toBeVisible();
  const orbitBounds = await orbitHint.boundingBox();
  const explodeBounds = await page.getByRole('button', { name: 'Explode robot' }).boundingBox();
  const resetBounds = await page.getByRole('button', { name: 'Reset 3D view' }).boundingBox();
  const exitBounds = await page.getByRole('button', { name: 'Exit 3D view' }).boundingBox();
  const finalIndexBounds = await page.locator('.component-index').boundingBox();
  expect(orbitBounds).not.toBeNull();
  expect(explodeBounds).not.toBeNull();
  expect(resetBounds).not.toBeNull();
  expect(exitBounds).not.toBeNull();
  expect(finalIndexBounds).not.toBeNull();
  if (orbitBounds && explodeBounds && resetBounds && exitBounds && finalIndexBounds) {
    expect(orbitBounds.x).toBeGreaterThanOrEqual(0);
    expect(orbitBounds.x + orbitBounds.width).toBeLessThanOrEqual(390);
    expect(finalIndexBounds.y + finalIndexBounds.height).toBeLessThanOrEqual(orbitBounds.y);
    expect(orbitBounds.y + orbitBounds.height).toBeLessThanOrEqual(explodeBounds.y);
    expect(explodeBounds.x + explodeBounds.width).toBeLessThanOrEqual(resetBounds.x);
    expect(resetBounds.x + resetBounds.width).toBeLessThanOrEqual(exitBounds.x);
    expect(exitBounds.x + exitBounds.width).toBeLessThanOrEqual(390);
  }
});

test('switches themes, avoids a black dark mode, and remembers the choice', async ({ page }) => {
  await routeModelToProceduralFallback(page);
  await page.goto('/');

  const autoScrollButton = page.getByRole('button', { name: 'Start auto scroll' });
  const themeButton = page.getByRole('button', { name: 'Switch to light mode' });
  const autoScrollBox = await autoScrollButton.boundingBox();
  const themeBox = await themeButton.boundingBox();
  expect(themeBox?.width).toBe(autoScrollBox?.width);
  expect(themeBox?.height).toBe(autoScrollBox?.height);

  await expect(page.locator('html')).toHaveAttribute('data-theme', 'dark');
  await expect.poll(() => page.locator('body').evaluate((body) => getComputedStyle(body).backgroundColor))
    .toMatch(/^rgba?\(74, 81, 84(?:, 1)?\)$/);

  await page.getByRole('button', { name: 'Switch to light mode' }).click();
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'light');

  await page.reload();
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'light');
  await expect(page.getByRole('button', { name: 'Switch to dark mode' })).toBeVisible();
});
