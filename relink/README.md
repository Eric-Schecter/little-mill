# OpenCAMLib source and relinking

The Web extension statically incorporates OpenCAMLib under LGPL-2.1-or-later.
The exact vendored OpenCAMLib source used by this build is in
`opencamlib-source.tar.gz`; `boost-headers-1.85.0.tar.gz` provides its header
build dependency. License copies are in `../site/licenses/`.

The `extension.cpp.o` application object and the four `.a` archives allow the
module to be relinked with a modified OpenCAMLib. `extension.cpp` and
`surface_deviation.hpp` are also supplied for inspection. Component versions
and the distributed WASM hash are recorded in `../BUILD.json`.

1. Install/activate official Emscripten **4.0.20**.
2. Run `sh build-ocl.sh` once to unpack/rebuild OpenCAMLib. Edit files under
   `work/opencamlib/src` before rebuilding its objects/archive as desired.
3. Run `sh relink.sh` to produce `cnc_manifold_web.new.wasm`.
4. Replace `../site/cnc_manifold_web.wasm`, serve the site, and close/reopen old
   browser sessions or clear this site's cached resources so the replacement
   is loaded. Keep the threaded/shared-memory ABI.

You may modify and relink the LGPL library in this demo and reverse engineer
the combined executable for debugging those modifications. These permissions
are not restricted by any additional project-specific terms. No permission to
restrict rights granted by the component licenses is asserted.
