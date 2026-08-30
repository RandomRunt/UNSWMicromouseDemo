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

  const controllerButton = page.getByRole('button', { name: 'MCU' });
  await expect(controllerButton).toBeVisible();
  await controllerButton.focus();
  await page.keyboard.press('Enter');

  await expect(page.getByTestId('component-detail')).toContainText('Controller');
  await expect(controllerButton).toHaveAttribute('aria-pressed', 'true');
});

test('unlocks 360-degree controls only at the bottom of the final chapter', async ({ page }) => {
  await page.goto('/');
  const canvas = page.getByTestId('showcase-canvas');

  await page.evaluate(() => {
    const maxScroll = document.documentElement.scrollHeight - window.innerHeight;
    window.scrollTo(0, maxScroll * 0.9);
  });
  await expect(canvas).toHaveAttribute('data-exploration-enabled', 'false');
  await expect(page.getByTestId('chapter-explore')).toBeInViewport();

  await page.evaluate(() => {
    const maxScroll = document.documentElement.scrollHeight - window.innerHeight;
    window.scrollTo(0, maxScroll);
  });
  await expect(canvas).toHaveAttribute('data-exploration-enabled', 'true');
});

test('has no automatically detectable critical accessibility violations', async ({ page }) => {
  await page.goto('/');
  const results = await new AxeBuilder({ page }).analyze();
  expect(results.violations.filter((violation) => violation.impact === 'critical')).toEqual([]);
});

test('keeps the primary story readable on a mobile viewport', async ({ page }) => {
  await page.goto('/');
  const openingHeading = page.getByTestId('chapter-meet').getByRole('heading');
  await expect(openingHeading).toBeVisible();
  await expect(openingHeading).toHaveText(/\S+/);

  const finalChapter = page.getByTestId('chapter-explore');
  await finalChapter.scrollIntoViewIfNeeded();
  const finalHeading = finalChapter.getByRole('heading');
  await expect(finalHeading).toBeVisible();
  await expect(finalHeading).toHaveText(/\S+/);
});

test('switches themes, avoids a black dark mode, and remembers the choice', async ({ page }) => {
  await page.goto('/');
  await page.evaluate(() => window.localStorage.setItem('micromouse-theme', 'dark'));
  await page.reload();

  await expect(page.locator('html')).toHaveAttribute('data-theme', 'dark');
  await expect.poll(() => page.locator('body').evaluate((body) => getComputedStyle(body).backgroundColor))
    .toBe('rgb(74, 81, 84)');

  await page.getByRole('button', { name: 'Switch to light mode' }).click();
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'light');

  await page.reload();
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'light');
  await expect(page.getByRole('button', { name: 'Switch to dark mode' })).toBeVisible();
});
