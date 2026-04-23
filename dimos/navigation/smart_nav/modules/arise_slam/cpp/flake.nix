{
  description = "arise_slam — LiDAR-inertial SLAM with ICP scan matching and factor graph optimization";

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
    sophus-src = {
      url = "github:strasdat/Sophus/1.22.10";
      flake = false;
    };
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, gtsam-src, sophus-src, lcm-extended, ... }:
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
            "-DGTSAM_BUILD_UNSTABLE=ON"
            "-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF"
            "-DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF"
            "-DGTSAM_INSTALL_CYTHON_TOOLBOX=OFF"
            "-DGTSAM_USE_SYSTEM_EIGEN=ON"
            "-DGTSAM_WITH_TBB=OFF"
            "-DBoost_NO_BOOST_CMAKE=OFF"
          ];
        };

        sophus = pkgs.stdenv.mkDerivation {
          pname = "sophus";
          version = "1.22.10";
          src = sophus-src;

          nativeBuildInputs = with pkgs; [ cmake ];
          buildInputs = with pkgs; [ eigen ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DSOPHUS_INSTALL=ON"
            "-DBUILD_TESTS=OFF"
            "-DBUILD_EXAMPLES=OFF"
          ];
        };
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "arise-slam";
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
            pkgs.ceres-solver
            pkgs.glog
            pkgs.gflags
            gtsam
            sophus
          ];

          cmakeFlags = [
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
          ];

          installPhase = ''
            mkdir -p $out/bin
            cp arise_slam $out/bin/
          '';
        };
      }
    );
}
