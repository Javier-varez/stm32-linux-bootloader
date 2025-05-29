{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
  };

  outputs =
    { nixpkgs, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      forAllSystems = nixpkgs.lib.genAttrs systems;

      shellForSystem =
        system:
        let
          pkgs = import nixpkgs { inherit system; };
        in
        (pkgs.buildFHSEnv {
          name = "buildenv";
            targetPkgs = pkgs: [
              pkgs.bash
              pkgs.wget
              pkgs.gnutar
              pkgs.cmake
              pkgs.libz
              pkgs.ncurses5
              pkgs.pkg-config
              pkgs.ninja
            ];
            runScript = "nu";

            # You might need to set LD_LIBRARY_PATH=/lib or ncurses5 will not be found
        }).env;

    in
    {
      devShell = forAllSystems shellForSystem;
    };
}
