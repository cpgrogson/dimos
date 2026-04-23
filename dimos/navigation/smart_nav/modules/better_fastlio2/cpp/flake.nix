{
  description = "BetterFastLio2 — enhanced FAST-LIO2 with dynamic removal and loop closure";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    gtsam-src = {
      url = "github:borglab/gtsam/develop";
      flake = false;
    };
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, gtsam-src, lcm-extended, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.default;

        gtsam = pkgs.stdenv.mkDerivation {
          pname = "gtsam";
          version = "4.2";
          src = gtsam-src;

          nativeBuildInputs = with pkgs; [ cmake pkg-config ];
          buildInputs = with pkgs; [ eigen boost tbb ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DCMAKE_POLICY_DEFAULT_CMP0167=OLD"
            "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF"
            "-DGTSAM_BUILD_TESTS=OFF"
            "-DGTSAM_BUILD_UNSTABLE=OFF"
            "-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF"
            "-DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF"
            "-DGTSAM_INSTALL_CYTHON_TOOLBOX=OFF"
            "-DGTSAM_USE_SYSTEM_EIGEN=ON"
            "-DGTSAM_WITH_TBB=OFF"
            "-DBoost_NO_BOOST_CMAKE=OFF"
          ];
        };
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "better-fastlio2";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = with pkgs; [
            cmake
            pkg-config
          ];

          buildInputs = [
            lcm
            pkgs.eigen
            pkgs.pcl
            pkgs.boost
            pkgs.tbb
            pkgs.glib
            pkgs.llvmPackages.openmp
            gtsam
          ];

          cmakeFlags = [
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
          ];

          installPhase = ''
            mkdir -p $out/bin
            cp better_fastlio2 $out/bin/
          '';
        };
      }
    );
}
