{
  description = "Hunt 2025 Radio";

  inputs.flake-utils.url = "github:numtide/flake-utils";
  inputs.nixpkgs-esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";

  outputs = { self, nixpkgs, flake-utils, nixpkgs-esp-dev }: let
    overlay = (final: prev: {
      radio-esp-idf = (prev.esp-idf-esp32s3.override {
        pkg-config = final.pkg-config-unwrapped;
      }).overrideAttrs (old: {
        patches = (old.patches or []) ++ [
          (final.fetchpatch {
            url = "https://github.com/espressif/esp-adf/raw/master/idf_patches/idf_v5.3_freertos.patch";
            hash = "sha256-8EfnrOv4aJrlJJnoRzxx7tzXbMXpxFx53cAGZFxEm6g=";
          })
        ];
      });
      esp-adf = final.fetchFromGitHub {
        owner = "espressif";
        repo = "esp-adf";
        fetchSubmodules = true;
        rev = "ff7f39dcf0a0da87671ff4ab422eb258be429049";
        hash = "sha256-C8kQ2AaAeQtOjt9BiGraXAiOlgtaVsXLnIIu9lZA88M=";
      };
    });
  in
    (flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nixpkgs-esp-dev.overlays.default
            overlay
          ];
        };
      in {
        packages = rec {
          inherit (pkgs) radio-esp-idf esp-adf;
          #default = bluechips-rs;
        };
        devShells.default = pkgs.mkShell {
          name = "default";
          buildInputs = with pkgs; [
            radio-esp-idf
          ];
          shellHook = ''
            export ADF_PATH=${pkgs.esp-adf}
          '';
        };
      })) // {
        overlays.default = overlay;
      };
}
