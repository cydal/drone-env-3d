"""Design language: a small palette + material helper. Slightly futuristic autonomous-ops facility."""
PALETTE = {
    "ground":       "0.36 0.40 0.36 1",   # dry grass / gravel
    "asphalt":      "0.16 0.17 0.19 1",
    "apron":        "0.44 0.46 0.48 1",   # concrete apron
    "concrete":     "0.62 0.64 0.66 1",
    "concrete_dk":  "0.48 0.50 0.53 1",
    "facade":       "0.70 0.74 0.78 1",   # light composite facade
    "facade_dk":    "0.30 0.34 0.40 1",   # dark glass band
    "steel":        "0.42 0.48 0.56 1",   # warehouse cladding
    "steel_dk":     "0.26 0.30 0.36 1",
    "roof":         "0.24 0.26 0.29 1",
    "accent":       "0.98 0.55 0.10 1",   # safety orange
    "accent2":      "0.16 0.70 0.96 1",   # ops cyan
    "white":        "0.92 0.93 0.94 1",
    "marking":      "0.95 0.95 0.90 1",
    "fence":        "0.55 0.58 0.60 0.85",
    "container_a":  "0.72 0.24 0.20 1",
    "container_b":  "0.20 0.42 0.62 1",
    "container_c":  "0.84 0.66 0.16 1",
    "tank":         "0.80 0.82 0.84 1",
    "hill":         "0.42 0.47 0.38 1",
    "glass":        "0.55 0.72 0.85 0.9",
    "beacon":       "0.95 0.20 0.20 1",
}


def material(key: str, *, emissive: str | None = None) -> str:
    rgba = PALETTE.get(key, key)
    em = f"<emissive>{emissive}</emissive>" if emissive else ""
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.15 0.15 0.15 1</specular>{em}</material>"
