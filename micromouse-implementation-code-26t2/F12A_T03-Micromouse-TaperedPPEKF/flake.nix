{
  description = "F12A_T03 Micromouse (firmware + high-level Python tools)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};

        pythonEnv = pkgs.python3.withPackages (ps: with ps; [
          numpy
          opencv4
          pyyaml
          scipy
        ]);

        planner = pkgs.writeShellApplication {
          name = "maze-planner";
          runtimeInputs = [ pythonEnv ];
          text = ''
            cd "${self}/high_level"
            exec python3 main.py "$@"
          '';
        };
      in
      {
        devShells.default = pkgs.mkShell {
          packages = [ pythonEnv ];
          shellHook = ''
            echo "Micromouse dev shell"
            echo "  high_level:  cd high_level  && python3 main.py"
            echo "  (also writes occupancy for every maze*/cylinder* under outputs/all/)"
            echo "  fast single: python3 main.py --skip-all"
            echo "  replan:      python3 main.py --path-only"
            echo "  re-export:   python3 main.py --export-only [--mode cylinders3]"
            echo "  Or: nix run .#maze-planner"
          '';
        };

        apps.default = {
          type = "app";
          program = "${planner}/bin/maze-planner";
        };

        apps.maze-planner = {
          type = "app";
          program = "${planner}/bin/maze-planner";
        };

        packages.default = planner;
        packages.maze-planner = planner;
      });
}
