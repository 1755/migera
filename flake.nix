{
  description = "Migera — Bevy game development shell";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, rust-overlay }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ rust-overlay.overlays.default ];
        config.allowUnfree = true;
      };

      # Bevy 0.19.1 requires Rust 1.95; use stable to stay on the latest stable toolchain
      rustToolchain = pkgs.rust-bin.stable.latest.default.override {
        extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ];
        targets = [ "x86_64-unknown-linux-gnu" "wasm32-unknown-unknown" ];
      };

      # Libraries Bevy links against at runtime (LD_LIBRARY_PATH)
      runtimeLibs = with pkgs; [
        vulkan-loader
        wayland
        wayland-protocols
        libxkbcommon
        libx11
        libxi
        libxcb
        libxcursor
        libxrandr
        alsa-lib
        udev
        stdenv.cc.cc.lib
      ];

      # Libraries needed by the Gothic export tool's Python venv (numpy's
      # compiled wheel needs libstdc++/zlib; zenkit's ctypes bindings need 3.10-3.13)
      pythonForTools = pkgs.python312;
      pythonToolsLibs = with pkgs; [
        stdenv.cc.cc.lib
        zlib
      ];

      # General-purpose Python for the default shell: ad-hoc analysis and
      # verification scripts (quaternion/rotation checks, plotting bench or
      # gait curves, glTF inspection, BRP JSON-RPC queries, PDF extraction).
      # Separate from `pythonForTools` above, which zenkit pins to <=3.13.
      pythonDev = pkgs.python3.withPackages (ps: with ps; [
        numpy
        scipy        # spatial.transform.Rotation: independent quat/euler reference
        matplotlib   # plots to PNG (MPLBACKEND=Agg below; no display needed)
        pillow       # screenshot crops/diffs
        pygltflib    # glTF/GLB JSON + accessors
        trimesh      # mesh loading/geometry queries
        pymupdf      # PDF text/page render from Python (docs/books/)
        requests     # Bevy Remote Protocol at 127.0.0.1:15702
      ]);

      # Build-time inputs
      buildInputs = with pkgs; [
        alsa-lib
        alsa-lib.dev
        udev
        udev.dev
        libxkbcommon
        libx11
        libxi
        libxcb
        libxcursor
        libxrandr
        wayland
        wayland-protocols
        vulkan-loader
        vulkan-headers
        vulkan-validation-layers
        openssl
        openssl.dev
      ];

      nativeBuildInputs = with pkgs; [
        pkg-config
        mold           # faster linker
        clang          # needed by bindgen (some bevy deps)
        cmake
      ];

    in {
      devShells.${system} = {
        default = pkgs.mkShell {
          inherit buildInputs nativeBuildInputs;

          packages = [
            rustToolchain
            pkgs.cargo-watch
            pkgs.cargo-expand
            pkgs.just
            pkgs.claude-code
            pkgs.opencode

            # GPU/shader profiling & debugging (Vulkan/RADV — this project's
            # wgpu backend). renderdoc: frame capture + dispatch timing +
            # compiled-shader disassembly (qrenderdoc GUI, or renderdoccmd for
            # headless capture). radeontop: quick GPU utilization sanity check
            # without a full capture. vulkan-tools: vulkaninfo/vkcube for
            # driver/extension sanity checks.
            pkgs.renderdoc
            pkgs.radeontop
            pkgs.vulkan-tools

            # PDF reading/extraction (reference books in docs/books/).
            # poppler-utils: pdfinfo/pdftotext/pdftoppm/pdfimages/pdfseparate.
            # qpdf: split/merge/repair. mupdf-headless: mutool (fast text,
            # structured-text JSON, page render). tesseract: OCR for scanned
            # pages that have no text layer.
            pkgs.poppler-utils
            pkgs.qpdf
            pkgs.mupdf-headless
            pkgs.tesseract

            pythonDev
          ];

          # Headless plotting: save figures to files instead of opening a window
          MPLBACKEND = "Agg";

          # Bevy's wgpu/winit need to find Vulkan and Wayland/X11 at runtime
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath runtimeLibs;

          # Use mold as the linker for fast incremental builds
          CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER = "${pkgs.clang}/bin/clang";
          RUSTFLAGS = "-C link-arg=-fuse-ld=${pkgs.mold}/bin/mold";

          # Tell rust-bindgen where clang lives
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";

          # Point to the system openssl
          PKG_CONFIG_PATH = "${pkgs.openssl.dev}/lib/pkgconfig";

          shellHook = ''
            echo "Bevy dev shell — Rust $(rustc --version)"
            echo "Linker: mold $(mold --version)"
          '';
        };

        # Standalone shell for tools/gothic_export (Gothic II -> glTF mesh export).
        # Not wired into the default shell: it's a side tool, not part of the Bevy build.
        gothic-export = pkgs.mkShell {
          packages = [ pythonForTools ];

          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath pythonToolsLibs;

          shellHook = ''
            echo "Gothic export dev shell — Python $(python3 --version)"
            if [ ! -d tools/gothic_export/.venv ]; then
              echo "Creating venv in tools/gothic_export/.venv ..."
              python3 -m venv tools/gothic_export/.venv
              tools/gothic_export/.venv/bin/pip install --quiet -r tools/gothic_export/requirements.txt
            fi
            echo "Activate with: source tools/gothic_export/.venv/bin/activate"
          '';
        };

        # Standalone shell for converting a downloaded character model (e.g. an
        # FBX export from Mixamo.com, which only ever exports FBX -- no glTF
        # option) into glTF/GLB for `assets/models/`. Not wired into the
        # default shell: same "side tool" precedent as `gothic-export` above.
        # Blender 2.8+ ships both an FBX importer and a glTF 2.0 exporter as
        # built-in add-ons (no extra Nix packages needed for either format),
        # and can run the conversion fully headless via `blender --background
        # --python <script>` -- see `tools/convert_model.py`.
        model-convert = pkgs.mkShell {
          packages = [ pkgs.blender ];

          shellHook = ''
            echo "Model conversion dev shell — $(blender --version | head -n1)"
            echo "Convert FBX (or any Blender-importable format) -> glTF/GLB:"
            echo "  blender --background --python tools/convert_model.py -- <input> <output.glb>"
          '';
        };
      };
    };
}
