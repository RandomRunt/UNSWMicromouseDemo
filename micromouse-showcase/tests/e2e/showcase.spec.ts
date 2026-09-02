import AxeBuilder from '@axe-core/playwright';
import { expect, test } from '@playwright/test';

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

  for (const id of ['meet', 'inside', 'sense', 'think', 'move', 'explore']) {
    await expect(page.getByTestId(`chapter-${id}`)).toBeAttached();
  }

  expect(errors).toEqual([]);
});

test('supports keyboard-accessible component inspection', async ({ page }) => {
  await page.goto('/');
  await page.getByTestId('chapter-inside').scrollIntoViewIfNeeded();

  const indexToggle = page.getByRole('button', { name: /component index/i });
  await expect(indexToggle).toHaveAttribute('aria-expanded', 'false');
  await expect(page.getByTestId('component-hint')).toContainText('Select a component');
  await indexToggle.click();
  await expect(indexToggle).toHaveAttribute('aria-expanded', 'true');
  await expect(page.getByTestId('component-hint')).toHaveCount(0);

  const controllerButton = page.getByRole('button', { name: 'MCU', includeHidden: true });
  await expect(controllerButton).toBeVisible();
  await controllerButton.focus();
  await page.keyboard.press('Enter');

  await expect(page.getByTestId('component-detail')).toContainText('Microcontroller');
  await expect(controllerButton).toHaveAttribute('aria-pressed', 'true');
  await expect(page.getByTestId('component-hint')).toHaveCount(0);

  if (test.info().project.name === 'mobile-chromium') {
    await expect(indexToggle).toHaveAttribute('aria-expanded', 'false');
    await expect(controllerButton).toBeHidden();
  } else {
    await indexToggle.click();
    await expect(indexToggle).toHaveAttribute('aria-expanded', 'false');
    await expect(controllerButton).toBeHidden();
  }
});

test('enables 360-degree controls at the desktop page end or through mobile 3D view', async ({ page }) => {
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

test('dismisses the orbit controls hint after a drag or inward zoom', async ({ page }) => {
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
    await page.mouse.down();
    await page.mouse.move(centerX + 60, centerY + 20, { steps: 4 });
    await page.mouse.up();
  }
  await expect(page.getByTestId('orbit-hint')).toHaveCount(0);
  await expect(page.getByTestId('component-detail')).toHaveCount(0);

  if (test.info().project.name === 'mobile-chromium') return;

  await page.reload();
  await page.evaluate(() => {
    const maxScroll = document.documentElement.scrollHeight - window.innerHeight;
    window.scrollTo(0, maxScroll);
  });
  await expect(page.getByTestId('orbit-hint')).toContainText('Drag to orbit');

  const reloadedBounds = await canvas.boundingBox();
  expect(reloadedBounds).not.toBeNull();
  if (!reloadedBounds) return;
  await page.mouse.move(
    reloadedBounds.x + reloadedBounds.width * 0.65,
    reloadedBounds.y + reloadedBounds.height * 0.5,
  );
  await page.mouse.wheel(0, -300);
  await expect(page.getByTestId('orbit-hint')).toHaveCount(0);
});

test('has no automatically detectable critical accessibility violations', async ({ page }) => {
  await page.goto('/');
  const results = await new AxeBuilder({ page }).analyze();
  expect(results.violations.filter((violation) => violation.impact === 'critical')).toEqual([]);
});

test('keeps every chapter and overlay readable on iPhone-sized screens', async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== 'mobile-chromium', 'iPhone layout coverage only runs in the touch project');
  testInfo.setTimeout(60_000);

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
  const finalIndexBounds = await page.locator('.component-index').boundingBox();
  expect(orbitBounds).not.toBeNull();
  expect(explodeBounds).not.toBeNull();
  expect(finalIndexBounds).not.toBeNull();
  if (orbitBounds && explodeBounds && finalIndexBounds) {
    expect(orbitBounds.x).toBeGreaterThanOrEqual(0);
    expect(orbitBounds.x + orbitBounds.width).toBeLessThanOrEqual(390);
    expect(orbitBounds.y).toBeGreaterThanOrEqual(explodeBounds.y + explodeBounds.height);
    expect(finalIndexBounds.x + finalIndexBounds.width).toBeLessThanOrEqual(explodeBounds.x);
  }
});

test('switches themes, avoids a black dark mode, and remembers the choice', async ({ page }) => {
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
