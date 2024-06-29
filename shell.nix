let
	pkgs = import <nixpkgs> { allowUnfree = true; };
in pkgs.mkShell {
	buildInputs = with pkgs; [
		steam-run
		jetbrains.idea-community
		jdk17
		glfw
		opengl
		# glx
	];
}


