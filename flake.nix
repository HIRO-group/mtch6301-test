{
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            (final: prev: {
              platformio-core = prev.platformio-core.overrideAttrs (
                finalAttrs: prevAttrs: {
                  propagatedBuildInputs =
                    prevAttrs.propagatedBuildInputs
                    ++ (with prev.python3Packages; [
                      pip
                      rich-click
                      pyyaml
                      typing-extensions
                      wheel
                    ]);
                  patches = [
                    (prev.replaceVars
                      (prev.fetchpatch {
                        url = "https://raw.githubusercontent.com/NixOS/nixpkgs/refs/heads/master/pkgs/development/embedded/platformio/interpreter.patch";
                        hash = "sha256-qTJCNr95S/DVSLigsFDZyy8/Ch/ziiHjGG0lHjOsRSY=";
                      })
                      {
                        interpreter =
                          (prev.python3Packages.python.withPackages (_: finalAttrs.propagatedBuildInputs)).interpreter;
                      }
                    )
                  ] ++ (prev.lib.lists.drop 1 prevAttrs.patches);
                }
              );
              platformio = prev.platformio.override { platformio-core = final.platformio-core; };
            })
          ];
        };
      in
      {
        devShell = pkgs.mkShell {
          buildInputs = with pkgs; [ 
	          platformio
            picocom 
          ];
        };
      }
    );
}
