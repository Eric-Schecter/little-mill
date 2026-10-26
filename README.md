# Little Mill

Browser demo of a Godot manufacturing workbench: target model → process planning
→ simulated machining → result inspection.

**Demo:** https://eric-schecter.github.io/little-mill/

This repository contains the exported Web build, deployment configuration and
third-party license/relinking materials. It does not contain local SDKs, API
keys, development caches or historical manufacturing-plan baselines.

## Hosting

GitHub Pages source: **GitHub Actions**. The workflow uploads only `site/`.
The threaded Godot 4.7.2 build uses Godot's bundled service worker to add
cross-origin isolation headers on static hosts. The first visit prepares the
worker and automatically reloads once. Use a current desktop browser with WebGL2,
WebAssembly threads and service workers enabled. No native processing server is
required. The published URL is a repository subpath; all runtime assets use
relative URLs.

For local testing without special headers:

```sh
python3 -m http.server 8080 --directory site
```

Open http://localhost:8080/ and allow the initial preparation to complete.

## Current validation scope

The existing workbench UI, machine scene, fixtures, planning and machining code
are exported from Godot. The C++ processing module is WASM with a serial
Manifold backend, running from Godot background threads. Core boolean,
Clipper2 and OpenCAMLib smoke tests have matched desktop results. Web trial-check
optimizations have passed controlled and partial STEP comparisons. Complete
STEP-cycle performance/quality equivalence is still under validation; this is
an experimental simulation demo, not production machine-control software.

## Third-party components

See [license notices](site/licenses/index.html) and [relinking materials](relink/).
OpenCAMLib is LGPL-2.1-or-later; its corresponding source and the objects/static
archives needed to relink the extension are included. Other component notices
are preserved in `site/licenses/`. No blanket license is asserted here for
project-specific code; the LGPL debugging/relinking permissions described in
`relink/README.md` apply.

`BUILD.json` records component versions and SHA-256 hashes of this export.
