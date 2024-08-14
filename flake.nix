{
  description = "Hunt 2025 Radio";

  inputs.flake-utils.url = "github:numtide/flake-utils";
  inputs.nixpkgs-esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev/31ada76b5aa9c5fb7781a327dac74b731ed48521";

  outputs = { self, nixpkgs, flake-utils, nixpkgs-esp-dev }: let
    overlay = (final: prev: {
      radio-esp-idf = (prev.esp-idf-esp32s3.override {
        rev = "v5.2.2";
        sha256 = "sha256-I4YxxSGdQT8twkoFx3zmZhyLTSagmeLD2pygVfY/pEk=";
        toolsToInclude = [
          "xtensa-esp-elf"
          "esp32ulp-elf"
          "openocd-esp32"
          "xtensa-esp-elf-gdb"
        ];
        pkg-config = final.pkg-config-unwrapped;
      }).overrideAttrs (old: {
        patches = (old.patches or []) ++ [
          (final.fetchpatch {
            url = "https://github.com/espressif/esp-adf/raw/master/idf_patches/idf_v5.2_freertos.patch";
            hash = "sha256-PlpG9qE0jC0+ltbqdIPrg89vUdMx7C946xX36cjdxDc=";
          })
        ];
      });
      esp-adf = final.fetchFromGitHub {
        owner = "espressif";
        repo = "esp-adf";
        fetchSubmodules = true;
        rev = "0593164bf7e64c268dad56e332d6f07f0e9c37ab";
        hash = "sha256-oVJHV5jd4lnRUanyQTpmqLgtA2627uSxpyIDwVgq1lk=";
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
