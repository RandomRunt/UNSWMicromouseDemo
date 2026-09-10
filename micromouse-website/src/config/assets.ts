// Vite changes BASE_URL to the GitHub repository path for the Pages build.
// Local development and Docker keep using '/', so the same model URL works
// in every environment without hard-coding the hosting domain.
export const MICROMOUSE_MODEL_URL = `${import.meta.env.BASE_URL}models/micromouse.glb`;
