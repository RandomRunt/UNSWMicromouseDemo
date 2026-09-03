import { useEffect, useState } from 'react';
import { MOBILE_VIEW_BREAKPOINT } from '../config/responsive';

const MOBILE_VIEW_QUERY = `(max-width: ${MOBILE_VIEW_BREAKPOINT}px)`;

export function useMobileViewport() {
  const [isMobileViewport, setIsMobileViewport] = useState(
    () => window.matchMedia(MOBILE_VIEW_QUERY).matches,
  );

  useEffect(() => {
    const query = window.matchMedia(MOBILE_VIEW_QUERY);
    const update = () => setIsMobileViewport(query.matches);
    update();
    query.addEventListener('change', update);
    return () => query.removeEventListener('change', update);
  }, []);

  return isMobileViewport;
}
